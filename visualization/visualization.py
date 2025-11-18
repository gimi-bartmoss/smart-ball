import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D

HOST = "192.168.4.1"
PORT = 8080

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

# 3D arrow object
quiver_acc = None
quiver_gyro = None

def main():
    global quiver_acc, quiver_gyro

    # Establish TCP connection with the ESP32 device
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    print(f"Connected to ESP32 at {HOST}:{PORT}")

    # Set up Matplotlib figure with multiple subplots
    # 1. Acceleration (AX, AY, AZ)
    # 2. Gyroscope   (GX, GY, GZ)
    # 3. Temperature (T)
    
    plt.style.use("ggplot")

    # fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    # MODIFIED: Changed layout to GridSpec (3 rows x 2 columns)
    # Left Column (0): Time Series | Right Column (1): 3D Vectors
    fig = plt.figure(figsize=(14, 8))
    gs = GridSpec(3, 2, figure=fig, width_ratios=[2, 1])

    # --- Left Column: Time Series Plots ---

    # Acceleration subplot
    ax_acc_ts = fig.add_subplot(gs[0, 0])
    line_ax, = ax_acc_ts.plot(ax_data, label="AX", color='r')
    line_ay, = ax_acc_ts.plot(ay_data, label="AY", color='g')
    line_az, = ax_acc_ts.plot(az_data, label="AZ", color='b')
    ax_acc_ts.set_title("Acceleration Time Series")
    ax_acc_ts.set_ylabel("m/s^2")
    ax_acc_ts.legend(loc='upper left', fontsize='small')

    # Gyroscope subplot
    ax_gyro_ts = fig.add_subplot(gs[1, 0], sharex=ax_acc_ts)
    line_gx, = ax_gyro_ts.plot(gx_data, label="GX", color='r')
    line_gy, = ax_gyro_ts.plot(gy_data, label="GY", color='g')
    line_gz, = ax_gyro_ts.plot(gz_data, label="GZ", color='b')
    ax_gyro_ts.set_title("Gyroscope Time Series")
    ax_gyro_ts.set_ylabel("deg/s")
    ax_gyro_ts.legend(loc='upper left', fontsize='small')

    # Temperature subplot
    ax_temp_ts = fig.add_subplot(gs[2, 0], sharex=ax_acc_ts)
    line_temp, = ax_temp_ts.plot(temp_data, label="Temp", color='orange')
    ax_temp_ts.set_title("Temperature")
    ax_temp_ts.set_ylabel("°C")
    ax_temp_ts.legend(loc='upper left', fontsize='small')

    # --- Right Column: 3D Vector Plots ---

    # 3D Acceleration Vector Plot
    ax_acc_3d = fig.add_subplot(gs[0:1, 1], projection='3d')
    ax_acc_3d.set_xlim([-20, 20])
    ax_acc_3d.set_ylim([-20, 20])
    ax_acc_3d.set_zlim([-20, 20])
    ax_acc_3d.set_xlabel('X')
    ax_acc_3d.set_ylabel('Y')
    ax_acc_3d.set_zlabel('Z')
    ax_acc_3d.set_title("Accel Vector")

    # 3D Gyroscope Vector Plot
    ax_gyro_3d = fig.add_subplot(gs[1:3, 1], projection='3d')
    ax_gyro_3d.set_xlim([-500, 500])
    ax_gyro_3d.set_ylim([-500, 500])
    ax_gyro_3d.set_zlim([-500, 500])
    ax_gyro_3d.set_xlabel('X')
    ax_gyro_3d.set_ylabel('Y')
    ax_gyro_3d.set_zlabel('Z')
    ax_gyro_3d.set_title("Gyro Vector")

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
        global quiver_acc, quiver_gyro

        raw = s.recv(1024)
        if not raw:
            return

        line = raw.decode().strip()
        imu = parse_imu(line)

        # Extract current values
        c_ax = imu.get("AX", 0)
        c_ay = imu.get("AY", 0)
        c_az = imu.get("AZ", 0)
        c_gx = imu.get("GX", 0)
        c_gy = imu.get("GY", 0)
        c_gz = imu.get("GZ", 0)
        c_t  = imu.get("T", 0)

        # Calculate Magnitude (Euclidean Norm)
        # Formula: sqrt(x^2 + y^2 + z^2)
        mag_acc = np.sqrt(c_ax**2 + c_ay**2 + c_az**2)
        mag_gyro = np.sqrt(c_gx**2 + c_gy**2 + c_gz**2)

        # Shift buffers to the left and append new values at the end
        ax_data = np.roll(ax_data, -1); ax_data[-1] = c_ax
        ay_data = np.roll(ay_data, -1); ay_data[-1] = c_ay
        az_data = np.roll(az_data, -1); az_data[-1] = c_az
        gx_data = np.roll(gx_data, -1); gx_data[-1] = c_gx
        gy_data = np.roll(gy_data, -1); gy_data[-1] = c_gy
        gz_data = np.roll(gz_data, -1); gz_data[-1] = c_gz
        temp_data = np.roll(temp_data, -1); temp_data[-1] = c_t

        # Update plot data
        line_ax.set_ydata(ax_data)
        line_ay.set_ydata(ay_data)
        line_az.set_ydata(az_data)

        line_gx.set_ydata(gx_data)
        line_gy.set_ydata(gy_data)
        line_gz.set_ydata(gz_data)

        line_temp.set_ydata(temp_data)

        # Autoscale y-axis based on current data
        autoscale_axis(ax_acc_ts, [ax_data, ay_data, az_data], min_range=2.0)
        autoscale_axis(ax_gyro_ts, [gx_data, gy_data, gz_data], min_range=5.0)
        autoscale_axis(ax_temp_ts, [temp_data], min_range=2.0)

        # 3D Acceleration Vector
        # Remove the old arrow if it exists
        if quiver_acc: 
            quiver_acc.remove()
        # Draw new arrow starting from (0,0,0) to (ax, ay, az)
        quiver_acc = ax_acc_3d.quiver(0, 0, 0, c_ax, c_ay, c_az, color='blue', arrow_length_ratio=0.1)
        auto_scale_3d(ax_acc_3d, c_ax, c_ay, c_az, min_range=9.8)

        # Update title with magnitude
        ax_acc_3d.set_title(f"Accel Vector\nMag: {mag_acc:.2f} m/s^2", y=0.99)

        # 3D Gyroscope Vector
        if quiver_gyro: 
            quiver_gyro.remove()
        quiver_gyro = ax_gyro_3d.quiver(0, 0, 0, c_gx, c_gy, c_gz, color='red', arrow_length_ratio=0.1)
        auto_scale_3d(ax_gyro_3d, c_gx, c_gy, c_gz, min_range=10.0)

        # Update title with magnitude
        ax_gyro_3d.set_title(f"Gyro Vector\nMag: {mag_gyro:.2f} deg/s")

        return (
            line_ax, line_ay, line_az,
            line_gx, line_gy, line_gz,
            line_temp
        )

    # Autoscale axis helper function
    def autoscale_axis(axis, data_list, min_range=1.0, margin_ratio=0.1):
        data = np.concatenate(data_list)
        dmin = float(np.min(data))
        dmax = float(np.max(data))

        if not np.isfinite(dmin) or not np.isfinite(dmax):
            return

        if abs(dmax - dmin) < 1e-6:
            center = dmin
            half = min_range / 2.0
            axis.set_ylim(center - half, center + half)
        else:
            diff = dmax - dmin
            margin = diff * margin_ratio
            axis.set_ylim(dmin - margin, dmax + margin)

    # Autoscale 3D vector helper function
    def auto_scale_3d(ax_3d, x, y, z, min_range=1.0):
        max_val = max(abs(x), abs(y), abs(z))
        
        limit = max(max_val, min_range) * 1.2
        
        ax_3d.set_xlim(-limit, limit)
        ax_3d.set_ylim(-limit, limit)
        ax_3d.set_zlim(-limit, limit)

    # Start animation loop
    ani = FuncAnimation(fig, update, interval=50)
    plt.tight_layout()
    plt.show()

    # Clean up connection
    s.close()

if __name__ == "__main__":
    main()