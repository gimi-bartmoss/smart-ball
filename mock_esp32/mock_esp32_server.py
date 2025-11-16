import socket
import time
import random

HOST = "0.0.0.0"  # Listen on all interfaces
PORT = 8080

# Function to generate random mock IMU data
def generate_mock_imu():
    ax = random.uniform(-5, 15)
    ay = random.uniform(-3, 3)
    az = random.uniform(-5, 5)
    gx = random.uniform(-5, 5)
    gy = random.uniform(-30, 0)
    gz = random.uniform(-5, 5)
    t  = random.uniform(28.0, 33.0)

    return f"AX:{ax:.2f}, AY:{ay:.2f}, AZ:{az:.2f}, GX:{gx:.2f}, GY:{gy:.2f}, GZ:{gz:.2f}, T:{t:.2f}"

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"[Mock ESP32] Listening on {HOST}:{PORT}")

    # Wait for a connection
    conn, addr = s.accept()
    print(f"[Mock ESP32] Connected by {addr}")

    with conn:
        while True:
            imu_line = generate_mock_imu()

            # Send the IMU data line
            line = imu_line + "\r\n"
            conn.send(line.encode())

            print("[Mock ESP32 Sent]", imu_line)
            time.sleep(0.5)
