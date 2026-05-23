# KukaPy

A Python library for controlling KUKA robots over TCP via KUKA's EthernetKRL (EKI) interface — inspired by [fanucpy](https://github.com/torayeff/fanucpy).

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python Version](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)

---

## Overview

KukaPy lets you send motion commands, read joint/Cartesian positions, and control digital outputs from a Python script running on a PC. Communication uses KUKA EthernetKRL (EKI) over TCP.

**Who is server / who is client:**

| Role | Side |
|---|---|
| **TCP Server** | Python (your PC) — listens on a port |
| **TCP Client** | KRC (KUKA Robot Controller) — connects out to Python |

The KRL program `KUKAPY_SERVER.SRC` runs on the pendant and initiates the connection. Your Python script calls `robot.connect()` to listen and wait for the KRC to connect in.

---

## Project Structure

```
kukapy/           Python library
  robot.py        Robot class — connect, move, read positions, digital I/O
  robotapp.py     High-level application helpers
  transformations.py  Rotation utilities (quaternion, Euler)

kukadriver/       Files to deploy on the KRC
  KUKAPY_SERVER.SRC   KRL server program (run this on the pendant)
  KUKAPY.xml          EKI channel configuration

kuka_example/
  test_ping.py        Minimal ping/pong connectivity test
  connection_test.py  Motion test — joint moves + Cartesian LIN move
```

---

## Requirements

- Python 3.8+
- KUKA KRC with KSS 8.x and EthernetKRL (EKI) option
- Network connectivity between PC and KRC

---

## Installation

```bash
git clone https://github.com/JasonLvernex/KukaPy.git
cd KukaPy
pip install -e .
```

No additional dependencies beyond the Python standard library.

---

## Deploying to the KRC

### 1. Copy KRL files

Copy both files from `kukadriver/` to the KRC at:

```
C:\KRC\ROBOTER\Config\User\Common\EthernetKRL\KUKAPY.xml
C:\KRC\ROBOTER\STEU\Mada\KUKAPY_SERVER.SRC   (or via teach pendant file manager)
```

### 2. Configure KUKAPY.xml

Open `KUKAPY.xml` and set `<IP>` to the **PC's IP address** as seen from the KRC network:

```xml
<EXTERNAL>
  <IP>192.168.0.1</IP>   <!-- PC IP reachable from KRC -->
  <PORT>18735</PORT>
  <PROTOCOL>TCP</PROTOCOL>
  <SENDFLAG>1</SENDFLAG>
</EXTERNAL>
```

**Cold-restart the KRC** after changing the XML so EKI reloads the configuration.

### 3. EKI configuration notes (important for simulation)

The following settings are required for correct operation — deviating from them causes `KSS01422` / `EKI000015` errors:

- **`Set_Flag="1"` must be on the last RECEIVE element** (`Rob/DO_Val`), not the first.  
  EKI sets `$FLAG[1]` as soon as the flagged element is parsed. If this is the first element, subsequent elements have not yet entered the buffer and all `EKI_GetInt` / `EKI_GetReal` calls fail with "empty receive buffer".

- **All KRL variables must be explicitly initialised before `EKI_Get*` calls.**  
  KRL marks locally declared variables as invalid until assigned. `EKI_GetInt` refuses to write into an invalid variable. `KUKAPY_SERVER.SRC` initialises all `cmd_*` variables to `0` / `0.0` after `EKI_OPEN`.

- **`SENDFLAG=1`** causes the KRC to append `\x00` to every `EKI_SEND` frame. The Python client handles both null-byte delimited and plain XML frames automatically.

### 4. Network setup for KUKA Sim Pro + VMware

If running KRC inside a VMware VM (e.g. OfficeLite):

- KRC's VxWorks real-time OS only routes `192.168.0.0/24`.
- Python runs on the Windows host, typically on a `192.168.253.x` VMware NAT interface.
- Bridge the two with a **portproxy** rule run as Administrator on the host:

```powershell
netsh portproxy add v4tov4 `
  listenaddress=192.168.0.1 listenport=18735 `
  connectaddress=192.168.253.1 connectport=18735
```

Also open the port in Windows Firewall:

```powershell
netsh advfirewall firewall add rule `
  name="KukaPy EKI" dir=in action=allow protocol=TCP localport=18735
```

Run `KUKAPY_SERVER.SRC` on the pendant **after** Python is listening.

---

## Quick Start

```python
from kukapy.robot import Robot

robot = Robot(port=18735)
robot.connect()          # blocks until KRC connects in

print(robot.ping())                          # → 'pong'
print(robot.get_curjpos())                   # [A1, A2, A3, A4, A5, A6] in degrees
print(robot.get_curpos())                    # [X, Y, Z, A, B, C] in mm / degrees

robot.move("joint", [0, -90, 90, 0, 90, 0], velocity=20)   # PTP joint move
robot.move("pose",  [500, 0, 800, 0, 0, 0], velocity=10, linear=True)  # LIN move

robot.set_do(1, True)                        # set digital output 1 high
print(robot.get_do(1))                       # read digital output 1

robot.disconnect()
```

### Robot class reference

| Method | Description |
|---|---|
| `connect()` | Listen on `port` and wait for KRC to connect |
| `disconnect()` | Send quit command and close socket |
| `ping()` | Round-trip check, returns `'pong'` |
| `get_curjpos()` | Current joint angles `[A1..A6]` in degrees |
| `get_curpos()` | Current TCP position `[X, Y, Z, A, B, C]` in mm / degrees |
| `move(move_type, vals, velocity, ...)` | Move robot (see below) |
| `set_do(num, value)` | Set digital output |
| `get_do(num)` | Read digital output state |
| `print_pos()` | Print joint and Cartesian position to stdout |

**`move()` parameters:**

| Parameter | Description |
|---|---|
| `move_type` | `"joint"` / `"movej"` — PTP joint move; `"pose"` / `"movep"` — Cartesian move |
| `vals` | 6-element list: `[A1..A6]` deg for joint, `[X,Y,Z,A,B,C]` mm/deg for Cartesian |
| `velocity` | Program override 1–100 % |
| `cnt_val` | Approximate positioning 0–100 % (0 = stop exactly) |
| `linear` | `True` = LIN move, `False` = PTP (Cartesian mode only) |

### Constructor options

```python
Robot(
    port=18735,
    socket_timeout=60,   # seconds to wait for KRC to connect
    recv_timeout=None,   # seconds to wait for command response (None = infinite)
)
```

Use `recv_timeout=300` (or similar) if long motion commands risk being cut off.

---

## Running the examples

```bash
# 1. Start KUKAPY_SERVER on the pendant
# 2. Run the ping test
python kuka_example/test_ping.py

# 3. Run the motion test (joint moves + Cartesian LIN)
python kuka_example/connection_test.py
```

---

## License

MIT License — see [LICENSE](LICENSE).
