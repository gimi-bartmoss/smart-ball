import socket
import time
import argparse
from random_generator import generate_random_imu
from real_generator import generate_real_imu

HOST = "0.0.0.0"
PORT = 8080

def run_mock_server(data_source="random", interval=0.5):
    generator = generate_random_imu if data_source == "random" else generate_real_imu

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"[Mock ESP32] Listening on {HOST}:{PORT} (mode={data_source})")

        # Wait for a connection
        conn, addr = s.accept()
        print(f"[Mock ESP32] Connected by {addr}")

        with conn:
            while True:
                imu_line = generator()
                conn.send((imu_line + "\r\n").encode())
                print(f"[Mock ESP32 Sent] {imu_line}")
                time.sleep(interval)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mock ESP32 IMU Data Server")
    parser.add_argument("--mode", choices=["random", "real"], default="random",
                        help="Select IMU data source: random (default) or real")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Data sending interval in seconds (default=0.5)")
    args = parser.parse_args()

    run_mock_server(data_source=args.mode, interval=args.interval)
