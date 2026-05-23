# KukaPy

A Python library for controlling KUKA robots over TCP via KUKA's EthernetKRL (EKI) interface — inspired by [fanucpy](https://github.com/torayeff/fanucpy).

[![MIT License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python Version](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)

---

## Overview

KukaPy lets you send motion commands, read joint/Cartesian positions, and control digital outputs from a Python script running on a PC. Communication uses KUKA EthernetKRL (EKI) over TCP.

### Control flow

```
  PC (Python)                          KRC (Robot Controller)
  ─────────────────────────────────────────────────────────────
  robot.connect()   ◄─── TCP connect ──── EKI_OPEN("KUKAPY")

  robot.move(...)   ────── XML cmd ─────► EKI receives frame
                                          KRL executes motion
  move() returns    ◄───── XML resp ───── EKI_SEND response
```

**Key point — who is server / who is client:**

| Role | Side |
|---|---|
| **TCP Server** | Python (your PC) — listens on a port, waits for KRC to connect |
| **TCP Client** | KRC — EKI initiates the outbound connection to Python |

This means Python must call `robot.connect()` **before** starting `KUKAPY_SERVER` on the pendant. The KRC connects out to the PC; the PC never connects to the KRC.

### Command set

Each Python call maps to a command code sent in the `<Type>` field:

| Code | Python method | KRL action |
|---|---|---|
| 1 | `ping()` | responds `pong` |
| 2 | `get_curjpos()` | reads `$AXIS_ACT` → returns A1–A6 |
| 3 | `get_curpos()` | reads `$POS_ACT` → returns X,Y,Z,A,B,C |
| 4 | `move("joint", ...)` | PTP to joint target |
| 5 | `move("pose", ...)` | PTP or LIN to Cartesian target |
| 6 | `set_do(n, v)` | sets `$OUT[n]` |
| 7 | `get_do(n)` | reads `$OUT[n]` |
| 8 | `disconnect()` | KRL exits loop, closes EKI channel |

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

- Python 3.8+ (no third-party dependencies — standard library only)
- KUKA KRC with KSS 8.x and EthernetKRL (EKI) option
- Network connectivity between PC and KRC

---

## Installation

**Install directly from GitHub (no cloning needed):**

```bash
pip install git+https://github.com/JasonLvernex/KukaPy.git
```

**Or clone and install in editable mode (for development):**

```bash
git clone https://github.com/JasonLvernex/KukaPy.git
cd KukaPy
pip install -e .
```

## Recommend to run in Administrator Mode
<img width="798" height="798" alt="126712805f4629695c003a0111d27da1" src="https://github.com/user-attachments/assets/dd0fa57f-5169-425b-bd77-1a5ae6392d0e" />

---

## Deploying to the KRC

### 1. Copy KRL files

Two files need to go to different locations on the KRC filesystem:

**`KUKAPY.xml` — EKI channel configuration**

```
C:\KRC\ROBOTER\Config\User\Common\EthernetKRL\KUKAPY.xml
```
<img width="1719" height="897" alt="6335c33dab842d1c262b6d4fb91326bb" src="https://github.com/user-attachments/assets/28b06c50-564d-4bb1-a739-baa95328b77b" />


EKI reads this file at `EKI_INIT("KUKAPY")` time. The filename (without extension) must match the string passed to `EKI_INIT` / `EKI_OPEN` in the KRL program.

**`KUKAPY_SERVER.SRC` — KRL server program**

Copy via USB stick or network share to the KRC program directory, then select and load it from the teach pendant. In KUKA Sim Pro you can drag-and-drop into the project tree.

### 2. Configure KUKAPY.xml

Open `KUKAPY.xml` and set `<IP>` to the PC's IP address reachable from the KRC, then set `<PORT>` to any free port. The other fields should stay as shown:

```xml
<EXTERNAL>
  <IP>YOUR_PC_IP</IP>
  <PORT>18735</PORT>
  <PROTOCOL>TCP</PROTOCOL>
  <SENDFLAG>1</SENDFLAG>
</EXTERNAL>
```


How to find `YOUR_PC_IP` depends on your setup — see Section 4 below.

**Cold-restart the KRC** after changing the XML so EKI reloads the configuration.

### 3. EKI configuration notes (important for simulation)

The following settings are required for correct operation — deviating from them causes `KSS01422` / `EKI000015` errors:

- **`Set_Flag="1"` must be on the last RECEIVE element** (`Rob/DO_Val`), not the first.  
  EKI sets `$FLAG[1]` as soon as the flagged element is parsed. If this is the first element, subsequent elements have not yet entered the buffer and all `EKI_GetInt` / `EKI_GetReal` calls fail with "empty receive buffer".

- **All KRL variables must be explicitly initialised before `EKI_Get*` calls.**  
  KRL marks locally declared variables as invalid until assigned. `EKI_GetInt` refuses to write into an invalid variable. `KUKAPY_SERVER.SRC` initialises all `cmd_*` variables to `0` / `0.0` after `EKI_OPEN`.

- **`SENDFLAG=1`** causes the KRC to append `\x00` to every `EKI_SEND` frame. The Python client handles both null-byte delimited and plain XML frames automatically.

### 4. Network setup

#### Option A — Real KRC (physical robot)

The PC and KRC must be on the same subnet, connected via Ethernet (directly or through a switch).

Run `ipconfig` on the PC and look for the adapter connected to the robot network. Use that IP in `KUKAPY.xml`:

```
Example: KRC is 192.168.1.100  →  PC adapter is 192.168.1.x  →  use that IP
```

No portproxy needed. Make sure Windows Firewall allows inbound TCP on the chosen port.

#### Option B — KUKA Sim Pro + VMware (OfficeLite)

The simulated KRC runs inside a VMware VM. Its VxWorks real-time OS can only route `192.168.0.0/24`. Python runs on the Windows host on a different subnet, so a bridge is required.

**Step 1 — Find your VMware host-only adapter IP**

Run `ipconfig` on the PC and find the `VMware Network Adapter VMnet8` entry:

<img width="1730" height="924" alt="image" src="https://github.com/user-attachments/assets/1fefc24e-b936-4443-ad8c-934f9f4521d2" />

```
Ethernet adapter VMware Network Adapter VMnet8:
    IPv4 Address: 192.168.253.1       ← this is YOUR host IP (VMnet8)
```

> The exact address depends on your VMware NAT settings and may differ from `192.168.253.1`. Use whatever `VMnet8` shows on your machine.

**Step 2 — Find the KRC's IP inside the VM**

Inside the VM, check the KRC network config. In OfficeLite it is typically `192.168.0.x`. You can also ping from the VM to confirm. The KRC will connect **out** to the PC, so you only need to know the PC side IP.
<img width="1719" height="897" alt="46b5fce89f3f31bb210ea4318a30cee8" src="https://github.com/user-attachments/assets/1dde5693-70ae-462c-95a5-07ba4c10c934" />


**Step 3 — Create a portproxy rule (run PowerShell as Administrator)**

This forwards traffic arriving at `192.168.0.1` (a virtual address the KRC can reach) to your real host IP on VMnet8:

```powershell
netsh portproxy add v4tov4 `
  listenaddress=192.168.0.1  listenport=18735 `
  connectaddress=192.168.253.1  connectport=18735
```

Replace `192.168.253.1` with your actual VMnet8 IP from Step 1.

Set `<IP>192.168.0.1</IP>` in `KUKAPY.xml` — this is the portproxy listen address, not a real network interface, so it will not appear in `ipconfig`.

**Step 4 — Open the port in Windows Firewall (run as Administrator)**

```powershell
netsh advfirewall firewall add rule `
  name="KukaPy EKI" dir=in action=allow protocol=TCP localport=18735
```

**Step 5 — Start order**

1. Run Python script first (`robot.connect()` starts listening)
2. Then start `KUKAPY_SERVER` on the pendant — KRC will connect out to the portproxy address

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
