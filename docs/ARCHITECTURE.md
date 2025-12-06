# Smart Ball Motion Tracking System

## Abstract

This project aims to develop a smart ball motion analysis system based on IoT technology. The core of the system utilizes an ESP32-C3 microcontroller and an MPU-6050 6-axis Inertial Measurement Unit (IMU) to transmit real-time motion data via Wi-Fi TCP/IP protocols. The backend software, developed in Python, integrates data relay, logging, and 3D trajectory reconstruction algorithms based on a Strapdown Inertial Navigation System (SINS). To mitigate the integral drift inherent in inertial navigation caused by sensor errors, the system incorporates a Zero Velocity Update (ZUPT) technique, achieving high-precision restoration of motion trajectories.

## System Architecture

The system consists of three main subsystems: the Embedded Sensing Layer, the Network Transmission Layer, and the Application Layer.

- Sensing Layer (Firmware): Responsible for sensor initialization, data sampling, and packet encapsulation.
- Transmission Layer (Network): Establishes a SoftAP hotspot and transmits binary data streams via TCP Sockets.
- Application Layer (Software): Includes a data relay server, a GUI-based logger, and tools for 2D/3D visualization and analysis.

## Hardware Implementation Details

### Microcontroller (MCU): Espressif ESP32-C3 SuperMini (RISC-V Architecture)

- Selection Rationale: Built-in Wi-Fi stack, compact size suitable for embedding in a ball, and low power consumption.

### Inertial Measurement Unit (IMU): InvenSense MPU-6050

- Specifications: 3-axis accelerometer and 3-axis gyroscope.
- Configuration: Accelerometer range $\pm16g$, Gyroscope range $\pm2000^\circ/s$ (defined in SensorModule.cpp).
- Interface: I2C (SDA: GPIO 8, SCL: GPIO 9).

## Firmware Design & Communication Protoco

The firmware is built using the PlatformIO development environment and the Arduino Framework.

### Modular Software Design

Object-Oriented Programming (OOP) is employed to enhance code maintainability:

- SensorModule: Encapsulates the driver logic for the MPU-6050. It initializes the I2C bus, sets the filter bandwidth (260Hz), and reads raw acceleration and angular velocity data.

- WiFiServerModule: Manages network operations. It configures the ESP32 in Access Point mode (SSID: SmartBall_AP) and starts a TCP Server (Port 8080) listening for client connections.

### Data Structure and Protocol

To ensure transmission efficiency, the system uses compact Binary Packets instead of text-based formats (like JSON).

The data structure defined in SensorModule.h is as follows:

```c
struct IMUData {
    uint32_t timestamp; // Timestamp (4 bytes)
    float ax, ay, az;   // Acceleration (3 * 4 bytes)
    float gx, gy, gz;   // Angular Velocity (3 * 4 bytes)
    float temp;         // Temperature (4 bytes)
};
```

- Packet Size: 32 Bytes.
- Endianness: Little-Endian (ESP32 default).
- Sampling Rate: The main loop delay is set to 40ms, resulting in an approximate sampling rate of 25Hz.

## Software and Algorithms

### Data Relay Architecture

To overcome the limitations of point-to-point TCP connections, a data_relay.py script was developed.

- Function: Acts as middleware, connecting to the ESP32 on one side and opening a Localhost Server (Port 8081) on the other.
- Advantage: Allows the "Monitor," "Data Logger," and "Real-time Visualization" applications to consume the same sensor data stream simultaneously.

### Motion Trajectory Reconstruction

A complete Strapdown Inertial Navigation System (SINS) is implemented in `visualization_3d.py`.

#### Core Steps

1. Initial Alignment:Calculates the gravity vector using the mean acceleration while stationary. It computes the initial attitude quaternion to align the sensor coordinate system with the world coordinate system.

1. Attitude Update:

    - Integrates angular velocity data from the gyroscope.
    - Updates the current attitude using quaternion differential equations to avoid Gimbal Lock.

1. Frame Transformation & Gravity Compensation:

    - Projects the acceleration from the Body Frame to the World Frame using the current attitude quaternion.
    - Subtracts the gravity vector ($[0, 0, g]$) to obtain pure motion acceleration.

1. Integration:

    - First integration of pure motion acceleration yields velocity.
    - Second integration yields position.

#### Zero Velocity Update (ZUPT)

Due to sensor noise and integration errors, simple double integration leads to severe position drift over time. This system implements a linear ZUPT algorithm:

- Assumption: The ball returns to a static state at the end of the recording.
- Implementation: Calculates the residual velocity error at the end, assumes a linear accumulation of error over time, and subtracts this drift from the velocity data across the entire timeline to correct the position.

### Data Visualization

- Real-time Display (`visualization.py`): Uses Matplotlib FuncAnimation to plot real-time waveforms of 3-axis acceleration, angular velocity, and temperature.

- 3D Analysis (`visualization_3d.py`)

  - Plots the comparison between the corrected trajectory and the uncorrected (Raw Integration) trajectory.
  - Displays acceleration changes in the World Frame.
  - Reconstructs the ball's rotational attitude via a 3D Wireframe animation.

## Experimental Results

Analysis based on the system's visualization output:

- Projectile Motion Analysis:

  The charts show the corrected trajectory (Blue line) versus the raw integrated trajectory (Orange dotted line). The ZUPT algorithm successfully eliminates the divergent drift common in inertial navigation, accurately restoring the parabolic path.

- Pure Rotation Motion:

  In the stationary rotation test, the corrected 3D trajectory remains near the origin, and the 3D animation on the right accurately reflects the ball's rotation axis and speed.

- Stationary State:

  Despite sensor noise, the velocity norm returns to zero after ZUPT processing,
  demonstrating the algorithm's effectiveness in stationary detection.

## Conclusion

This project successfully integrates embedded systems, wireless communication, and inertial navigation algorithms. By combining the ESP32 and MPU-6050 hardware with backend Python SINS and ZUPT algorithms, a low-cost 3D motion tracking system with reasonable accuracy was achieved. The binary transmission protocol and relay architecture ensure real-time data transmission and multi-client scalability, laying a solid foundation for future motion analysis applications.

## Appendix: Mathematical Principles and Implementation Details

In a Strapdown Inertial Navigation System (SINS), attitude estimation is the foundation of trajectory reconstruction. This system uses Quaternions to represent and update the ball's attitude. Compared to Euler Angles, quaternions effectively avoid the Gimbal Lock problem and offer higher computational efficiency when synthesizing rotations.

The implementation logic is divided into two phases: **Initial Alignment** and **Strapdown Integration**. The relevant code is located in `tools/visualization_3d.py`.

### A. Sensor Error Model and Calibration

The inertial sensor measurements are modeled as the sum of the true value, a constant bias, and Gaussian white noise.

- Gyroscope Model:$$\boldsymbol{\omega}_{m} = \mathbf{S}_g \boldsymbol{\omega}_{true} + \mathbf{b}_g + \boldsymbol{\eta}_g$$

- Accelerometer Model:$$\mathbf{a}_{m} = \mathbf{S}_a \mathbf{a}_{true} + \mathbf{b}_a + \boldsymbol{\eta}_a$$

Where $b$ represents the bias and $η$ represents noise. During the static initialization phase $0 \le t \le T_{static}$, the gyroscope bias $\hat{\mathbf{b}}_g$ is estimated by averaging the raw readings, assuming $\boldsymbol{\omega}_{true} \approx \mathbf{0}$:$$\hat{\mathbf{b}}_g = \frac{1}{N} \sum_{k=1}^{N} \boldsymbol{\omega}_{m}(t_k)$$

This bias is subtracted from all subsequent readings: $\boldsymbol{\omega} = \boldsymbol{\omega}_{m} - \hat{\mathbf{b}}_g$

### B. Initial Alignment

During the initial system startup, the gravity vector in the static state is used to infer the ball's initial attitude. The goal is to calculate a rotation quaternion $q_{init}$ such that the gravity vector measured by the sensor (Body Frame) aligns with the World Frame gravity vector ($[0, 0, g]$).

1. Calculate the mean acceleration vector $\vec{a}_{static}$ during the static period.
1. Define the World Frame gravity vector $\vec{g}_{world} = [0, 0, |\vec{a}_{static}|]$.
1. Calculate the rotation axis $\vec{u}$ using the Cross Product: $$ \vec{u} = \frac{\vec{a}{static} \times \vec{g}{world}}{||\vec{a}{static} \times \vec{g}{world}||} $$
1. Calculate the rotation angle $\theta$ using the Dot Product: $$ \theta = \arccos\left( \frac{\vec{a}{static} \cdot \vec{g}{world}}{||\vec{a}{static}|| \cdot ||\vec{g}{world}||} \right) $$
1. Construct the initial quaternion using the axis and angle.

Implemented using the scipy.spatial.transform.Rotation library:

```py
# [source: tools/visualization_3d.py]

# 1. Calculate static mean acceleration (Body Frame)
acc_static_mean = df[['AX', 'AY', 'AZ']].iloc[:n_static].mean().values

# 2. Define World Frame gravity vector
local_gravity_magnitude = np.linalg.norm(acc_static_mean)
world_gravity = np.array([0, 0, local_gravity_magnitude])

# Normalize vectors
acc_static_norm = acc_static_mean / np.linalg.norm(acc_static_mean)
world_g_norm = world_gravity / np.linalg.norm(world_gravity)

# 3. Compute rotation axis (Cross Product) and angle (Dot Product)
rot_axis = np.cross(acc_static_norm, world_g_norm)
rot_angle = np.arccos(np.dot(acc_static_norm, world_g_norm))

# 4. Create initial rotation quaternion
# Handle the case where vectors are already aligned (axis length ~ 0)
if np.linalg.norm(rot_axis) < 1e-6:
    r_align = R.identity()
else:
    r_align = R.from_rotvec(rot_axis / np.linalg.norm(rot_axis) * rot_angle)

# Initialize current attitude
q_current = r_align
```

### C. Attitude Update

During motion, the system uses angular velocity measured by the gyroscope to update the attitude quaternion.

1. Assuming the mean angular velocity vector over time step $\Delta t$ is $\vec{\omega} = [\omega_x, \omega_y, \omega_z]$:
1. Calculate the rotation angle magnitude: $\Delta \theta = ||\vec{\omega}|| \cdot \Delta t$.
1. Calculate the rotation axis unit vector: $\vec{n} = \frac{\vec{\omega}}{||\vec{\omega}||}$.
1. Construct the Delta Quaternion (Incremental Rotation) $\Delta q$: $$ \Delta q = \left[ \cos\left(\frac{\Delta \theta}{2}\right), \vec{n} \sin\left(\frac{\Delta \theta}{2}\right) \right] $$
1. Update the current attitude $q_{k+1}$ (Quaternion Multiplication):
$$ q_{k+1} = q_{k} \otimes \Delta q $$

The following logic is executed for each sample point within the integration loop:

```py
# [source: tools/visualization_3d.py]

# Get angular velocity and subtract bias
omega = gyro_data[i] - gyro_bias

# Calculate rotation magnitude for this time step (rad)
angle_magnitude = np.linalg.norm(omega) * dt

if angle_magnitude > 0:
    # Calculate rotation axis
    axis = omega / np.linalg.norm(omega)
    
    # Create incremental rotation quaternion
    # R.from_rotvec handles the conversion from axis-angle to quaternion
    r_delta = R.from_rotvec(axis * angle_magnitude)
    
    # Update global attitude: q_new = q_old * q_delta
    # Note: SciPy's multiplication order corresponds to the sequence of coordinate transformations
    q_current = q_current * r_delta
```

### D. Coordinate Transformation

After obtaining the updated attitude quaternion $q_{current}$, the acceleration measured in the Body Frame ($\mathbf{a}_{body}$) is projected into the World Frame ($\mathbf{a}_{world}$) for gravity compensation and position integration.

Acceleration is transformed from the Body Frame ($B$) to the World Frame ($W$) using the current attitude $q_k$​:$$\mathbf{a}_{world} = q_k \otimes \mathbf{a}_{body} \otimes q_k^{-1}$$

The pure motion acceleration amotion​ is obtained by removing the gravity component:$$\mathbf{a}_{motion} = \mathbf{a}_{world} - \mathbf{g}_{world}$$

```py
# [source: tools/visualization_3d.py]

# Specific Force Projection (Body Frame -> World Frame)
acc_body = acc_data[i]

# Rotate acceleration vector to World Frame using current quaternion
acc_world = q_current.apply(acc_body)

# Gravity Compensation
# Subtract gravity vector to get pure motion acceleration
acc_motion = acc_world - world_gravity
```

### E. Discrete-Time Numerical Integration

To reconstruct velocity and position, the system employs numerical integration. While simple Euler integration is often used, Trapezoidal integration offers reduced discretization error.

Let $\Delta t$ be the sampling interval between time step $k-1$ and $k$.

Velocity Update:

$$\mathbf{v}_k = \mathbf{v}_{k-1} + \frac{1}{2}(\mathbf{a}_k + \mathbf{a}_{k-1}) \cdot \Delta t$$

Position Update:

$$\mathbf{p}_k = \mathbf{p}_{k-1} + \frac{1}{2}(\mathbf{v}_k + \mathbf{v}_{k-1}) \cdot \Delta t$$

The following Python snippet demonstrates the Trapezoidal integration loop used to calculate velocity and position from the gravity-compensated acceleration data (acc_motion).

```py
# [source: tools/visualization_3d.py]
import numpy as np

# Initialize arrays for velocity and position
velocity = np.zeros((n_samples, 3))
position = np.zeros((n_samples, 3))

for i in range(1, n_samples):
    # Calculate time delta (dt)
    dt = timestamp[i] - timestamp[i-1]
    
    # 1. Velocity Integration (Trapezoidal Rule)
    # v_now = v_prev + 0.5 * (a_now + a_prev) * dt
    velocity[i] = velocity[i-1] + 0.5 * (acc_motion[i] + acc_motion[i-1]) * dt
    
    # 2. Position Integration (Trapezoidal Rule)
    # p_now = p_prev + 0.5 * (v_now + v_prev) * dt
    position[i] = position[i-1] + 0.5 * (velocity[i] + velocity[i-1]) * dt
```

### F: Zero Velocity Update (ZUPT) Mathematical Formulation

The ZUPT algorithm operates on the assumption that the object is stationary at $t=0$ and $t=T$. Due to sensor noise accumulation, the uncorrected terminal velocity is non-zero ($\mathbf{v}_{raw}(T) \neq \mathbf{0}$).

We define the linear Drift Rate vector $\mathbf{D}$:$$\mathbf{D} = \frac{\mathbf{v}_{raw}(T) - \mathbf{v}_{true}(T)}{T} = \frac{\mathbf{v}_{raw}(T)}{T}$$

The corrected velocity $\mathbf{v}_{corr}(t_k)$ at any time step $t_k$ is calculated as:$$\mathbf{v}_{corr}(t_k) = \mathbf{v}_{raw}(t_k) - \mathbf{D} \cdot t_k$$

The final corrected position is obtained by integrating the corrected velocity:$$\mathbf{p}_{corr}(t_k) = \sum_{i=1}^{k} \mathbf{v}_{corr}(t_i) \cdot \Delta t$$

The code below calculates the linear drift based on the final velocity error and applies the correction across the entire timeline.

```py
# [source: tools/visualization_3d.py]

# 1. Calculate Linear Drift Rate
# Assumption: The ball is stationary at the end (True Velocity = 0)
v_final_error = velocity[-1]  # The uncorrected velocity at the last timestamp
total_duration = timestamp[-1] - timestamp[0]

# Drift Rate D = V_error / Total_Time
drift_rate = v_final_error / total_duration

# 2. Apply Correction to Velocity
velocity_corrected = np.zeros_like(velocity)

for i in range(n_samples):
    current_time = timestamp[i] - timestamp[0]
    
    # Subtract the accumulated linear error: v_corr = v_raw - (D * t)
    velocity_corrected[i] = velocity[i] - (drift_rate * current_time)

# 3. Re-integrate Position using Corrected Velocity
position_corrected = np.zeros_like(position)

for i in range(1, n_samples):
    dt = timestamp[i] - timestamp[i-1]
    
    # Standard Trapezoidal integration for position
    position_corrected[i] = position_corrected[i-1] + \
                            0.5 * (velocity_corrected[i] + velocity_corrected[i-1]) * dt
```
