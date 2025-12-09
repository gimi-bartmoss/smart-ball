# Smart Ball Motion Tracking System

This project is designed to capture, visualize, and analyze the motion of a smart ball equipped with an ESP32-C3 microcontroller and an MPU-6050 IMU sensor. The system streams real-time motion data over Wi-Fi, which is then processed by a suite of Python applications for data logging, 3D trajectory reconstruction, and visualization.

## System Architecture

The system is composed of three main layers:

1. **Embedded Sensing Layer (Firmware):** The ESP32-C3 firmware reads 6-axis data (3-axis acceleration, 3-axis angular velocity) from the MPU-6050 sensor, encapsulates it into binary packets, and transmits it.
2. **Network Transmission Layer (Wi-Fi):** The ESP32-C3 operates in Soft Access Point (SoftAP) mode, creating its own Wi-Fi network (`SSID: SmartBall_AP`). It runs a TCP server on port `8080` to stream the sensor data to a connected client.
3. **Application Layer (Software):** A set of Python tools on a host computer receives the data. This includes a data relay server, a GUI data logger, and scripts for real-time 2D/3D visualization and trajectory analysis.

### Data Flow

`[Smart Ball: ESP32 + MPU-6050]` -> `Wi-Fi (TCP)` -> `[Computer: Data Relay]` -> `[Multiple Python Clients]`

## Trajectory Reconstruction

The ball's 3D trajectory is reconstructed from the raw sensor data using a **Strapdown Inertial Navigation System (SINS)** algorithm.

1. **Attitude Estimation:** The gyroscope's angular velocity data is integrated to calculate the ball's orientation (attitude) in real-time, represented by quaternions to avoid issues like Gimbal Lock.
2. **Gravity Compensation:** The initial orientation is used to determine the direction of gravity. This gravitational force is then subtracted from the accelerometer readings to isolate the ball's pure linear motion acceleration.
3. **Integration:** The linear acceleration is integrated twice over time to compute the ball's velocity and, subsequently, its position.

To counteract the significant drift errors that accumulate during integration, the system implements a **Zero Velocity Update (ZUPT)** algorithm. This technique assumes the ball is stationary at the beginning and end of its motion, calculates the accumulated velocity error, and distributes the correction linearly across the entire timeline, resulting in a much more accurate trajectory.

## Hardware

- **Microcontroller:** ESP32-C3 SuperMini (RISC-V)
- **Sensor:** MPU-6050 (6-axis accelerometer and gyroscope)

## Firmware

The firmware for the ESP32-C3 is located in the `esp32-c3-supermini` directory and is built using **PlatformIO**.

### Key Components

- `SensorModule`: A C++ class that handles all interactions with the MPU-6050 sensor via the I2C bus.
- `WiFiServerModule`: A C++ class that manages the Wi-Fi SoftAP and TCP server setup.

### Data Protocol

To maximize efficiency, data is transmitted as a compact 32-byte binary packet at approximately 25 Hz.

```c
struct IMUData {
    uint32_t timestamp; // Timestamp (ms)
    float ax, ay, az;   // Acceleration (g)
    float gx, gy, gz;   // Angular Velocity (deg/s)
    float temp;         // Temperature (°C)
};
```

### Building and Flashing

1. Install [Visual Studio Code](https://code.visualstudio.com/) and the [PlatformIO IDE extension](https://platformio.org/platformio-ide).
2. Open the `esp32-c3-supermini` directory in VS Code.
3. Connect the ESP32-C3 to your computer via USB.
4. Use the PlatformIO "Upload" task to build and flash the firmware.

## Software

The host software, written in Python, provides the tools to receive, process, and visualize the smart ball's data.

### Requirements

- Python 3.7+
- NumPy
- Matplotlib
- SciPy
- Pyglet (for 3D visualization)

### Installation

Clone the repository and install the required Python packages:

```bash
git clone https://github.com/your-username/smart-ball.git
cd smart-ball
pip install numpy matplotlib scipy pyglet
```

### Usage

1. **Power On the Smart Ball:** The ESP32 will create a Wi-Fi network named `SmartBall_AP`.
2. **Connect to the Network:** Connect your computer to the `SmartBall_AP` Wi-Fi network.
3. **Run the Data Relay:** This script is the central hub for data. It connects to the ball and re-broadcasts the data on `localhost:8081`, allowing multiple applications to listen simultaneously.

    ```bash
    python tools/data_relay.py
    ```

4. **Run an Application (in a new terminal):**
    - **Monitor:** View the raw sensor data stream in the console.

        ```bash
        python tools/monitor.py
        ```

    - **Data Logger:** Save the incoming data to a CSV file in the `raw_data` directory.

        ```bash
        python tools/data_logger.py
        ```

    - **2D Visualization:** See real-time plots of the accelerometer and gyroscope data.

        ```bash
        python tools/visualization.py
        ```

    - **3D Visualization & Analysis:** Reconstruct and visualize the 3D trajectory of the ball's motion. This is the most comprehensive tool, showing the raw vs. corrected path.

        ```bash
        python tools/visualization_3d.py
        ```

## Example Visualizations

### Projectile Motion

The corrected trajectory (blue) vs. the uncorrected, drifting trajectory (orange).

![Projectile Motion](figure/projectile.png)

### Stationary Ball

Even when stationary, sensor noise can cause drift. ZUPT corrects for this.

![Stationary Ball](figure/stationary.png)

### Ball Rotation

The system accurately captures rotational motion, which is visualized in the 3D animation.

![Ball Rotation](figure/rotation.png)

## Directory Structure

```bash
.
├── demo/
│   └── main_app.py           # Demo application
├── docs/
│   └── ARCHITECTURE.md       # In-depth technical documentation
├── esp32-c3-supermini/
│   ├── platformio.ini        # PlatformIO project configuration
│   └── src/                  # Firmware source code (C++)
├── figure/
│   ├── projectile.png
│   ├── rotation.png
│   └── stationary.png
├── sample_data/                 # Directory for storing sample logged sensor data
└── tools/
    ├── analysis.py           # Post-processing and analysis scripts
    ├── client_test.py        # TCP client for testing
    ├── data_logger.py        # Logs sensor data to a file
    ├── data_relay.py         # Relays data from the ball to local clients
    ├── monitor.py            # Prints raw data to the console
    ├── visualization.py      # Real-time 2D data plotting
    └── visualization_3d.py   # 3D trajectory reconstruction and visualization
```
