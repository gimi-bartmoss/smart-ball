import socket
import struct

HOST = "192.168.4.1"
PORT = 8080

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print(f"Connected to ESP32 at {HOST}:{PORT}")

try:
    while True:
        data = s.recv(32)
        if len(data) < 32:
            continue
        
        unpacked_data = struct.unpack('<I7f', data)
        timestamp, ax, ay, az, gx, gy, gz, temp = unpacked_data
        
        print(f"Timestamp: {timestamp}, AX: {ax:.2f}, AY: {ay:.2f}, AZ: {az:.2f}, GX: {gx:.2f}, GY: {gy:.2f}, GZ: {gz:.2f}, Temp: {temp:.2f}")

except KeyboardInterrupt:
    print("\nDisconnected by user")
finally:
    s.close()