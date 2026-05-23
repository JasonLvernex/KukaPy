import socket

KRC_IP = "192.168.253.128"
PORT   = 18735

print(f"Connecting to KRC at {KRC_IP}:{PORT}...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(30)
try:
    s.connect((KRC_IP, PORT))
    print(f"CONNECTED to KRC!")
    print("EKI Server mode works — TCP host->KRC OK")
    s.close()
except Exception as e:
    print(f"FAILED: {e}")