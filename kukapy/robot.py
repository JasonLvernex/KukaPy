from __future__ import annotations

import socket
import xml.etree.ElementTree as ET
from typing import Literal


class KukaError(Exception):
    pass


class Robot:
    """Python client for a KUKA robot running KUKAPY_SERVER.SRC via EKI.

    EKI acts as TCP client (KRC connects OUT to Python).
    Call connect() first — it listens on self.port until KRC connects.
    Then start KUKAPY_SERVER on the teach pendant.

    Usage:
        robot = Robot(port=18735)
        robot.connect()          # blocks until KRC connects in
        robot.move("joint", [0, -90, 90, 0, 90, 0], velocity=10)
        robot.disconnect()
    """

    SUCCESS_CODE = 0
    ERROR_CODE = 1

    _CMD_CODES: dict[str, int] = {
        "ping":    1,
        "curjpos": 2,
        "curpos":  3,
        "movej":   4,
        "movep":   5,
        "setdo":   6,
        "getdo":   7,
        "quit":    8,
    }

    def __init__(
        self,
        host: str = "192.168.1.1",
        port: int = 18735,
        socket_timeout: int = 60,
        recv_timeout: int | None = None,
        # legacy alias kept for compatibility
        ip: str | None = None,
    ):
        self.host = ip if ip is not None else host
        self.port = port
        self.socket_timeout = socket_timeout  # used only while waiting for KRC to connect
        self.recv_timeout = recv_timeout       # None = wait forever for command responses
        self.comm_sock: socket.socket | None = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Listen on self.port and wait for KRC (EKI client) to connect in."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.port))
        srv.listen(1)
        srv.settimeout(self.socket_timeout)
        print(f"[kukapy] Listening on :{self.port} — start KUKAPY_SERVER on the pendant now...")
        try:
            self.comm_sock, addr = srv.accept()
        finally:
            srv.close()
        self.comm_sock.settimeout(self.recv_timeout)
        print(f"[kukapy] KRC connected from {addr[0]}:{addr[1]}")

    def disconnect(self) -> None:
        try:
            self._send_cmd("quit")
        except Exception:
            pass
        if self.comm_sock:
            self.comm_sock.close()
            self.comm_sock = None

    # ------------------------------------------------------------------
    # EKI XML framing helpers
    # KUKAPY.xml uses PROTOCOL=TCP (raw stream, no null-byte framing).
    # We detect message boundaries by attempting incremental XML parsing.
    # ------------------------------------------------------------------

    def _send_xml(self, element: ET.Element) -> None:
        xml_str = ET.tostring(element, encoding="unicode")
        data = xml_str.encode("utf-8")
        #print(f"[kukapy] SEND {repr(data)}")
        self.comm_sock.sendall(data)

    def _recv_xml(self) -> ET.Element:
        """Read one <Res> frame.
        Handles both SENDFLAG=1 (\\x00-delimited) and plain XML modes.
        AliveCounter / other non-Res frames are silently skipped."""
        buf = b""
        while True:
            chunk = self.comm_sock.recv(4096)
            if not chunk:
                raise KukaError("Connection closed by KRC")
            buf += chunk
            #print(f"[kukapy] RECV {repr(buf)}")

            # Mode 1: \x00-delimited frames (SENDFLAG=1 on KRC side)
            while b"\x00" in buf:
                frame, buf = buf.split(b"\x00", 1)
                frame = frame.strip()
                if not frame:
                    continue
                try:
                    elem = ET.fromstring(frame.decode("utf-8"))
                    if elem.tag == "Res":
                        return elem
                except ET.ParseError:
                    pass

            # Mode 2: plain XML stream (no SENDFLAG) — try to parse what we have
            stripped = buf.strip()
            if stripped:
                try:
                    elem = ET.fromstring(stripped.decode("utf-8"))
                    if elem.tag == "Res":
                        buf = b""
                        return elem
                    buf = b""  # non-Res complete frame, discard and keep reading
                except ET.ParseError:
                    pass  # incomplete XML, accumulate more

    def _build_cmd(self, cmd_type: str, **kw) -> ET.Element:
        """Build a <Rob> XML element with all fields populated."""
        rob = ET.Element("Rob")
        ET.SubElement(rob, "Type").text = str(self._CMD_CODES[cmd_type])
        ET.SubElement(rob, "Vel").text    = str(int(kw.get("Vel",    0)))
        ET.SubElement(rob, "Acc").text    = str(int(kw.get("Acc",    0)))
        ET.SubElement(rob, "Cnt").text    = str(int(kw.get("Cnt",    0)))
        ET.SubElement(rob, "Lin").text    = str(int(kw.get("Lin",    0)))
        ET.SubElement(rob, "A1").text     = f"{float(kw.get('A1', 0.0)):.6f}"
        ET.SubElement(rob, "A2").text     = f"{float(kw.get('A2', 0.0)):.6f}"
        ET.SubElement(rob, "A3").text     = f"{float(kw.get('A3', 0.0)):.6f}"
        ET.SubElement(rob, "A4").text     = f"{float(kw.get('A4', 0.0)):.6f}"
        ET.SubElement(rob, "A5").text     = f"{float(kw.get('A5', 0.0)):.6f}"
        ET.SubElement(rob, "A6").text     = f"{float(kw.get('A6', 0.0)):.6f}"
        ET.SubElement(rob, "X").text      = f"{float(kw.get('X',  0.0)):.6f}"
        ET.SubElement(rob, "Y").text      = f"{float(kw.get('Y',  0.0)):.6f}"
        ET.SubElement(rob, "Z").text      = f"{float(kw.get('Z',  0.0)):.6f}"
        ET.SubElement(rob, "OA").text     = f"{float(kw.get('OA', 0.0)):.6f}"
        ET.SubElement(rob, "OB").text     = f"{float(kw.get('OB', 0.0)):.6f}"
        ET.SubElement(rob, "OC").text     = f"{float(kw.get('OC', 0.0)):.6f}"
        ET.SubElement(rob, "DO_Num").text = str(int(kw.get("DO_Num", 0)))
        ET.SubElement(rob, "DO_Val").text = str(int(kw.get("DO_Val", 0)))
        return rob

    def _send_cmd(self, cmd_type: str, **kw) -> ET.Element:
        """Send command and return the raw <Res> response element."""
        self._send_xml(self._build_cmd(cmd_type, **kw))
        resp = self._recv_xml()
        code = int(resp.findtext("Code", default="1"))
        msg  = resp.findtext("Msg",  default="")
        if code != self.SUCCESS_CODE:
            raise KukaError(msg)
        return resp

    # ------------------------------------------------------------------
    # Robot API
    # ------------------------------------------------------------------

    def ping(self) -> str:
        """Round-trip check. Returns 'pong' on success."""
        self._send_cmd("ping")
        return "pong"

    def get_curjpos(self) -> list[float]:
        """Current joint angles [A1..A6] in degrees."""
        resp = self._send_cmd("curjpos")
        return [float(resp.findtext(f"A{i}", default="0")) for i in range(1, 7)]

    def get_curpos(self) -> list[float]:
        """Current TCP position [X, Y, Z, A, B, C] in mm / degrees."""
        resp = self._send_cmd("curpos")
        return [float(resp.findtext(k, default="0")) for k in ("X", "Y", "Z", "OA", "OB", "OC")]

    def move(
        self,
        move_type: Literal["joint", "movej", "pose", "movep"],
        vals: list[float],
        velocity: int = 25,
        acceleration: int = 100,
        cnt_val: int = 0,
        linear: bool = False,
        continue_on_error: bool = False,
    ) -> None:
        """Move the robot.

        Args:
            move_type:  "joint"/"movej"  →  PTP joint move  (vals = [A1..A6] deg)
                        "pose"/"movep"   →  cartesian move   (vals = [X,Y,Z,A,B,C])
            velocity:   program override 1-100 %.
            cnt_val:    approximate positioning 0-100 % (0 = stop exactly).
            linear:     (cartesian only) True = LIN move, False = PTP.
        """
        if not (1 <= velocity <= 100):
            raise ValueError("velocity must be 1-100 %")
        if not (0 <= cnt_val <= 100):
            raise ValueError("cnt_val must be 0-100")
        if len(vals) != 6:
            raise ValueError("vals must have exactly 6 elements")

        if move_type in ("joint", "movej"):
            kw = dict(Vel=velocity, Acc=acceleration, Cnt=cnt_val, Lin=0,
                      A1=vals[0], A2=vals[1], A3=vals[2],
                      A4=vals[3], A5=vals[4], A6=vals[5])
            cmd = "movej"
        elif move_type in ("pose", "movep"):
            kw = dict(Vel=velocity, Acc=acceleration, Cnt=cnt_val, Lin=int(linear),
                      X=vals[0], Y=vals[1], Z=vals[2],
                      OA=vals[3], OB=vals[4], OC=vals[5])
            cmd = "movep"
        else:
            raise ValueError(f"Unknown move_type: {move_type!r}")

        try:
            self._send_cmd(cmd, **kw)
        except KukaError:
            if not continue_on_error:
                raise

    def set_do(self, do_num: int, value: bool) -> None:
        """Set digital output."""
        self._send_cmd("setdo", DO_Num=do_num, DO_Val=int(value))

    def get_do(self, do_num: int) -> bool:
        """Get digital output state."""
        resp = self._send_cmd("getdo", DO_Num=do_num)
        return bool(int(resp.findtext("DO_Val", default="0")))

    def print_pos(self) -> None:
        """Print current joint angles and TCP position to stdout."""
        j = self.get_curjpos()
        p = self.get_curpos()
        print(f"  Joints    : A1={j[0]:.1f}  A2={j[1]:.1f}  A3={j[2]:.1f}  A4={j[3]:.1f}  A5={j[4]:.1f}  A6={j[5]:.1f}")
        print(f"  Cartesian : X={p[0]:.1f}  Y={p[1]:.1f}  Z={p[2]:.1f}  A={p[3]:.1f}  B={p[4]:.1f}  C={p[5]:.1f}")
