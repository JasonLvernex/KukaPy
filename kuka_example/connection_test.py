"""
KukaPy connection & motion test
================================
1. Copy KUKAPY.xml and KUKAPY_SERVER.SRC to the KRC.
2. Start KUKAPY_SERVER on the teach pendant.
3. Run this script.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from kukapy.robot import Robot

robot = Robot(port=18735, recv_timeout=300) # recv_timeout (s) upper-limt for runtime wait time
print("Waiting for KRC to connect...")
robot.connect()

print("\n--- Initial position ---")
robot.print_pos()

print("\n--- Move 1: Home (joint) ---")
robot.move("joint", [0, -90, 90, 0, 90, 0], velocity=100)
robot.print_pos()

print("\n--- Move 2: Cartesian +100 mm in Z (LIN) ---")
p = robot.get_curpos()
robot.move("pose", [p[0], p[1], p[2] + 300, p[3], p[4], p[5]], velocity=80, linear=True)
robot.print_pos()

print("\n--- Move 3: A1 -45 deg ---")
robot.move("joint", [-45, -90, 90, 0, 90, 0], velocity=100)
robot.print_pos()

print("\n--- Return to Home ---")
robot.move("joint", [0, -90, 90, 0, 90, 0], velocity=20)
robot.print_pos()

robot.disconnect()
print("\nDone.")
