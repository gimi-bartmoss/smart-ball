# Smart Ball Project

This project is designed to capture and visualize the motion of a smart ball equipped with an ESP32-C3 microcontroller and an MPU-6050 IMU sensor. The system allows for real-time data streaming over Wi-Fi, data logging, and 3D visualization of the ball's trajectory.

## Background Knowledge

The core of this project is the Inertial Measurement Unit (IMU). An IMU is an electronic device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers, gyroscopes, and sometimes magnetometers.

In this project, we use the MPU-6050, which contains a 3-axis accelerometer and a 3-axis gyroscope.

-   **Accelerometer**: Measures linear acceleration. When the ball is stationary, it measures the Earth's gravitational acceleration (~9.8 m/s²). When the ball is in motion, it measures the acceleration due to that motion combined with gravity.
-   **Gyroscope**: Measures angular velocity or the rate of rotation. This tells us how fast the ball is spinning around its X, Y, and Z axes.

By processing the data from these two sensors, we can reconstruct the ball's movement in 3D space.

## System Overview

The data flows from the smart ball to the computer for processing and visualization.

`[Smart Ball: ESP32 + MPU-6050]` -> `Wi-Fi (UDP)` -> `[Computer: Python Scripts]`

1.  The **MPU-6050** continuously measures acceleration and angular velocity.
2.  The **ESP32-C3** reads this data and streams it over a Wi-Fi network using the UDP protocol.
3.  A **Python script** on the computer listens for this UDP data stream. It can then be used to log the data, visualize it in real-time, or relay it to other applications.

## Measurement Principle

The trajectory of the ball is estimated by processing the raw sensor data from the MPU-6050.

1.  **Orientation**: The gyroscope data is used to track the orientation of the ball. By integrating the angular velocity over time, we can determine the ball's orientation (roll, pitch, yaw).
2.  **Acceleration Correction**: The initial orientation is used to subtract the effect of gravity from the accelerometer readings. This leaves us with the linear acceleration of the ball.
3.  **Trajectory Calculation**: By integrating the linear acceleration twice with respect to time, we can calculate the ball's velocity and then its position, effectively reconstructing its path of motion.

*   `Velocity = Initial Velocity + ∫(Linear Acceleration) dt`
*   `Position = Initial Position + ∫(Velocity) dt`

This process allows us to translate the raw electrical signals from the sensor into a meaningful 3D trajectory.

## Hardware

-   **Microcontroller:** ESP32-C3 SuperMini
-   **Sensor:** MPU-6050 6-axis accelerometer and gyroscope

## Firmware

The firmware for the ESP32-C3 is located in the `esp32-c3-supermini` directory. It is a PlatformIO project.

### Dependencies

-   Adafruit MPU6050
-   Adafruit Unified Sensor
-   Adafruit BusIO

### Building and Flashing

1.  Open the `esp32-c3-supermini` directory in Visual Studio Code with the PlatformIO extension.
2.  Connect the ESP32-C3 to your computer.
3.  Modify the Wi-Fi credentials in `src/main.cpp` to connect to your network.
4.  Build and upload the firmware using the PlatformIO interface.

## Software

The software components are located in the `tools`, `visualization`, and `visualization_3d` directories.

### Requirements

-   Python 3
-   NumPy
-   Matplotlib
-   Pyglet

### Installation

```bash
pip install numpy matplotlib pyglet
```

### Operation Flow

1.  **Start the ESP32**: Power on the smart ball. It will automatically connect to the configured Wi-Fi network and start broadcasting sensor data.

2.  **Run the Data Relay (Optional but Recommended)**: The `data_relay.py` script listens for data from the ESP32 and re-broadcasts it on `localhost`. This allows multiple client applications (like a logger and a visualizer) to access the data stream simultaneously.

    ```bash
    python tools/data_relay.py
    ```

3.  **Choose a Tool**:
    -   **Monitor**: To see the raw sensor data printed to the console, run:
        ```bash
        python tools/monitor.py
        ```
    -   **Data Logger**: To save the sensor data to a file for later analysis, run:
        ```bash
        python tools/data_logger.py
        ```
    -   **2D Visualization**: To see a simple 2D plot of the data, run:
        ```bash
        python visualization/visualization.py
        ```
    -   **3D Visualization**: To see a full 3D representation of the ball's trajectory, run:
        ```bash
        python visualization_3d/visualization_3d.py
        ```

## Example Visualizations

### Projectile Motion

![Projectile Motion](figure/projectile.png)

### Stationary Ball

![Stationary Ball](figure/stationary.png)

### Ball Rotation

![Ball Rotation](figure/rotation.png)

## Directory Structure

```
.
├── visualization_3d/     # 3D visualization scripts
├── esp32-c3-supermini/     # Firmware for the ESP32-C3
├── figure/                 # Figures for the README
├── mock_esp32/             # Mock ESP32 for testing
├── raw_data/               # Raw sensor data
├── tools/                  # Python scripts for data logging, relay, etc.
└── visualization/          # 2D visualization scripts
```