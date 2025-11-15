import time
import serial
import serial.tools.list_ports
import datetime

def find_port():
    """Find the first /dev/ttyACM* device."""
    while True:
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "ACM" in p.device:
                return p.device
        print("Waiting for ESP32-C3 USB CDC...")
        time.sleep(0.5)

def monitor(port):
    """Monitor serial output with automatic reconnect."""
    while True:
        try:
            print(f"\n[Connected to {port}]")
            with serial.Serial(port, 115200, timeout=0.1) as s:
                while True:
                    line = s.readline().decode(errors="ignore").strip()
                    if line:
                        print(datetime.datetime.now().isoformat(timespec="milliseconds"), line)  # Print timestamped line
                    else:
                        time.sleep(0.01)  # Avoid busy waiting
        except serial.SerialException:
            print("[Disconnected. Waiting for reconnection...]")
            port = find_port()

if __name__ == "__main__":
    try:
        port = find_port()
        monitor(port)
    except KeyboardInterrupt:
        print("\n[Stopped by user]")

