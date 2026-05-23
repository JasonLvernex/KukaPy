import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

s.bind(("0.0.0.0", 18735))
s.listen(1)

print("Listening on :18735 — start EKITEST now...")

s.settimeout(120)

try:
    conn, addr = s.accept()

    print(f"CONNECTED from {addr[0]}:{addr[1]}")

    input("Press Enter to close...")

    conn.close()

except socket.timeout:
    print("TIMEOUT — no connection arrived within 120s")

finally:
    s.close()