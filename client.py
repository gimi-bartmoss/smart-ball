import socket

HOST = "192.168.4.1"
PORT = 8080

s = socket.socket()
s.connect((HOST, PORT))

# Set threshold
s.sendall(b"SET THRESH=40.0\n")
print(s.recv(1024).decode().strip())

# Ask current threshold
s.sendall(b"GET THRESH\n")
print(s.recv(1024).decode().strip())

# Then wait to receive IMU data block after an impact
data = s.recv(65535)
print(data.decode(errors="ignore")[:500])  # print first 500 chars
