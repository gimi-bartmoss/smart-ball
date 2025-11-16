import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

HOST = "192.168.4.1"
PORT = 8080

# Range of plots
RANGE_ACC = 20   # m/s^2, [-RANGE_ACC, RANGE_ACC]
RANGE_GYRO = 50  # deg/s, [-RANGE_GYRO, RANGE_GYRO]
RANGE_TEMP = 50  # °C, [0, RANGE_TEMP]

# Number of samples to keep in the visualization buffer (FIFO)
N = 200

# Initialize data buffers for acceleration, gyroscope, and temperature
ax_data = np.zeros(N)
ay_data = np.zeros(N)
az_data = np.zeros(N)
gx_data = np.zeros(N)
gy_data = np.zeros(N)
gz_data = np.zeros(N)
temp_data = np.zeros(N)

def main():

    # Establish TCP connection with the ESP32 device
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print(f"Connected to ESP32 at {HOST}:{PORT}")

    # Set up Matplotlib figure with multiple subplots
    # 1. Acceleration (AX, AY, AZ)
    # 2. Gyroscope   (GX, GY, GZ)
    # 3. Temperature (T)
    plt.style.use("ggplot")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    # Acceleration subplot
    ax_acc = axes[0]
    line_ax, = ax_acc.plot(ax_data, label="AX")
    line_ay, = ax_acc.plot(ay_data, label="AY")
    line_az, = ax_acc.plot(az_data, label="AZ")
    ax_acc.set_ylim(-RANGE_ACC, RANGE_ACC)
    ax_acc.set_title("Real-time Acceleration")
    ax_acc.set_ylabel("m/s^2")
    ax_acc.legend()

    # Gyroscope subplot
    ax_gyro = axes[1]
    line_gx, = ax_gyro.plot(gx_data, label="GX")
    line_gy, = ax_gyro.plot(gy_data, label="GY")
    line_gz, = ax_gyro.plot(gz_data, label="GZ")
    ax_gyro.set_ylim(-RANGE_GYRO, RANGE_GYRO)
    ax_gyro.set_title("Real-time Gyroscope")
    ax_gyro.set_ylabel("deg/s")
    ax_gyro.legend()

    # Temperature subplot
    ax_temp = axes[2]
    line_temp, = ax_temp.plot(temp_data, label="Temperature (°C)")
    ax_temp.set_ylim(0, RANGE_TEMP)
    ax_temp.set_title("IMU Temperature")
    ax_temp.set_xlabel("Samples")
    ax_temp.set_ylabel("°C")
    ax_temp.legend()

    # Parse IMU ASCII data from ESP32
    def parse_imu(line: str) -> dict[str, float]:
        result = {}
        for p in line.split(','):
            if ':' not in p:
                continue
            key, *rest = p.split(':')
            if not rest:
                continue
            try:
                value = float(rest[0].strip())
                result[key.strip()] = value
            except ValueError:
                continue
        return result

    # Animation update function
    # Reads one line of IMU data and updates the visualization buffers
    def update(frame):
        global ax_data, ay_data, az_data
        global gx_data, gy_data, gz_data, temp_data

        raw = s.recv(1024)
        if not raw:
            return

        line = raw.decode().strip()
        imu = parse_imu(line)

        # Shift buffers to the left and append new values at the end
        ax_data = np.roll(ax_data, -1)
        ay_data = np.roll(ay_data, -1)
        az_data = np.roll(az_data, -1)
        gx_data = np.roll(gx_data, -1)
        gy_data = np.roll(gy_data, -1)
        gz_data = np.roll(gz_data, -1)
        temp_data = np.roll(temp_data, -1)

        ax_data[-1] = imu.get("AX", 0)
        ay_data[-1] = imu.get("AY", 0)
        az_data[-1] = imu.get("AZ", 0)
        gx_data[-1] = imu.get("GX", 0)
        gy_data[-1] = imu.get("GY", 0)
        gz_data[-1] = imu.get("GZ", 0)
        temp_data[-1] = imu.get("T", 0)

        # Update plot data
        line_ax.set_ydata(ax_data)
        line_ay.set_ydata(ay_data)
        line_az.set_ydata(az_data)

        line_gx.set_ydata(gx_data)
        line_gy.set_ydata(gy_data)
        line_gz.set_ydata(gz_data)

        line_temp.set_ydata(temp_data)

        return (
            line_ax, line_ay, line_az,
            line_gx, line_gy, line_gz,
            line_temp
        )

    # Start animation loop
    ani = FuncAnimation(fig, update, interval=50)
    plt.tight_layout()
    plt.show()

    # Clean up connection
    s.close()

if __name__ == "__main__":
    main()