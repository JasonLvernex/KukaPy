import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
from kukapy.robot import Robot

robot = Robot(port=18735)
print("Waiting for KRC to connect...")
robot.connect()
print("Connected! Sending ping...")
result = robot.ping()
print(f"Response: {result}")
robot.disconnect()
