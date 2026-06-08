"""
Minimal EKI GetInt test — matches EKIMIN.SRC / EKIMIN.xml.

Steps:
  1. Copy EKIMIN.xml and EKIMIN.SRC to the VM and cold-restart KRC.
  2. Start EKIMIN on the pendant (it will WAIT SEC 30).
  3. Run this script within that 30-second window.
  4. Check Res/Val:  1 = GetInt worked,  -1 = buffer was empty (GetInt failed).
"""
import socket
import xml.etree.ElementTree as ET
import sys
import os

PORT = 18735

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(('0.0.0.0', PORT))
srv.listen(1)
print(f"Listening on :{PORT} — start EKIMIN on pendant now (you have 30 s)...")
srv.settimeout(60)
conn, addr = srv.accept()
srv.close()
print(f"KRC connected from {addr}")

conn.settimeout(60)

# Send the minimal frame immediately
payload = b"<Rob><Type>1</Type></Rob>"
print(f"SEND {repr(payload)}")
conn.sendall(payload)

# Receive response (SENDFLAG=1 → \x00-delimited)
buf = b""
while True:
    chunk = conn.recv(4096)
    if not chunk:
        print("Connection closed before response.")
        sys.exit(1)
    buf += chunk
    print(f"RECV {repr(buf)}")
    if b"\x00" in buf:
        frame = buf.split(b"\x00")[0].strip()
        if frame:
            try:
                elem = ET.fromstring(frame.decode("utf-8"))
                code = elem.findtext("Code", "-99")
                val  = elem.findtext("Val",  "-99")
                print(f"\nRes/Code = {code}")
                print(f"Res/Val  = {val}  (expected 1 if GetInt worked, -1 if buffer was empty)")
                break
            except ET.ParseError as e:
                print(f"ParseError: {e}")
                break

conn.close()
