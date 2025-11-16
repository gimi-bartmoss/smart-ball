import socket
import time

HOST = "192.168.4.1"
PORT = 8080

s = socket.socket()
s.connect((HOST, PORT))

# Set threshold
s.sendall(b"SET THRESH=15.0\n")
print(s.recv(1024).decode().strip())

# Ask current threshold
s.sendall(b"GET THRESH\n")
print(s.recv(1024).decode().strip())

print("=== Waiting for IMU data... ===")

buffer = b""

while True:
    try:
        chunk = s.recv(4096)

        if chunk:
            buffer += chunk

            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                print(line.decode(errors="ignore"))
        else:
            time.sleep(0.02)

    except BlockingIOError:
        time.sleep(0.02)
        continue

    except ConnectionResetError:
        print("Connection reset by server.")
        break

    except KeyboardInterrupt:
        print("Exiting.")
        break
