import socket

HOST = "192.168.4.1"
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print(f"Connected to ESP32 at {HOST}:{PORT}")

try:
    while True:
        data = s.recv(1024)
        if not data:
            continue
        print(data.decode().strip())
except KeyboardInterrupt:
    print("\nDisconnected by user")
finally:
    s.close()
