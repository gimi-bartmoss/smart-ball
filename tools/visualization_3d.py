import os
import numpy as np
import pandas as pd
import argparse
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation

ani = None

def parse_data(filepath):
    """
    Parses the projectile data file.
    
    Args:
        filepath (str): Path to the text file.
        
    Returns:
        pd.DataFrame: Parsed dataframe with Time in seconds and dt.
    """
    data = []
    # Check if file exists before trying to open
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"The file '{filepath}' was not found. Please ensure the data file exists.")

    with open(filepath, 'r') as f:
        for line in f:
            if not line.strip(): continue
            parts = line.strip().split(', ')
            entry = {}
            for part in parts:
                key, value = part.split(': ')
                entry[key.strip()] = float(value)
            data.append(entry)
    
    df = pd.DataFrame(data)
    
    # Convert timestamp from milliseconds to seconds and normalize to start from 0
    df['Time'] = df['Timestamp'] / 1000.0
    df['Time'] = df['Time'] - df['Time'].iloc[0]
    
    # Calculate time delta (dt)
    df['dt'] = df['Time'].diff().fillna(0)
    
    return df

def calculate_kinematics(df):
    """
    Performs Strapdown Inertial Navigation System (SINS) calculation.
    Includes:
    1. Initial Alignment (Gravity direction detection)
    2. Attitude Update (using Quaternions)
    3. Gravity Compensation (Transforming Body Frame to World Frame)
    4. Zero Velocity Update (ZUPT) for linear drift correction
    """
    
    # --- 1. Initial Alignment ---
    # Use the first N static samples to determine the initial orientation.
    # The goal is to align the sensor's measured gravity vector (which points up, ~+9.8m/s^2 on Z)
    # with the World frame's gravity vector, which we define to also be [0, 0, +g].
    n_static = 20
    if len(df) < n_static:
        raise ValueError("Not enough data points for initial alignment.")

    acc_static_mean = df[['AX', 'AY', 'AZ']].iloc[:n_static].mean().values
    
    # Calculate gyro bias
    gyro_bias = df[['GX', 'GY', 'GZ']].iloc[:n_static].mean().values 
    
    # Measured gravity vector in Body Frame. We define world gravity to match the direction of the sensor's measurement.
    local_gravity_magnitude = np.linalg.norm(acc_static_mean)
    world_gravity = np.array([0, 0, local_gravity_magnitude])

    global gravity
    gravity = local_gravity_magnitude
    
    # Normalize vectors for calculation
    acc_static_norm = acc_static_mean / np.linalg.norm(acc_static_mean)
    world_g_norm = world_gravity / np.linalg.norm(world_gravity)
    
    # Compute the rotation axis (cross product) and angle (dot product) required to align the vectors
    rot_axis = np.cross(acc_static_norm, world_g_norm)
    rot_angle = np.arccos(np.dot(acc_static_norm, world_g_norm))
    
    # Create the initial rotation quaternion
    # Note: If the axis length is 0 (already aligned), handle gracefully.
    if np.linalg.norm(rot_axis) < 1e-6:
        r_align = R.identity()
    else:
        r_align = R.from_rotvec(rot_axis / np.linalg.norm(rot_axis) * rot_angle)
    
    # Initialize current attitude quaternion
    q_current = r_align
    
    # --- 2. Strapdown Integration Loop ---
    # Initialize arrays to store results
    velocities = np.zeros((len(df), 3))
    positions = np.zeros((len(df), 3))
    accelerations_world = np.zeros((len(df), 3))
    quaternions = np.zeros((len(df), 4)) # Store as [x, y, z, w]
    
    # State vectors
    v_curr = np.array([0.0, 0.0, 0.0])
    p_curr = np.array([0.0, 0.0, 0.0])
    
    # Store initial quaternion
    quaternions[0] = q_current.as_quat()

    # Extract numpy arrays for performance
    acc_data = df[['AX', 'AY', 'AZ']].values
    gyro_data = df[['GX', 'GY', 'GZ']].values  # Assumed rad/s
    dt_data = df['dt'].values
    
    # Loop through each timestamp (skipping the first as dt=0)
    for i in range(1, len(df)):
        dt = dt_data[i]
        
        # Safety check: If data gap is too large (e.g., packet loss), limit dt to avoid integration explosion.
        # In a real system, interpolation or Kalman Filter prediction would be used here.
        if dt > 1.0: 
            dt = 0.01 
        
        # A. Attitude Update
        # Integrate angular velocity to update orientation (Quaternion integration)
        omega = gyro_data[i] - gyro_bias
        angle_magnitude = np.linalg.norm(omega) * dt
        
        if angle_magnitude > 0:
            axis = omega / np.linalg.norm(omega)
            # Create incremental rotation quaternion
            r_delta = R.from_rotvec(axis * angle_magnitude)
            # Update global attitude: q_new = q_old * q_delta
            q_current = q_current * r_delta
        
        # Store current orientation
        quaternions[i] = q_current.as_quat()

        # B. Specific Force Projection (Body Frame -> World Frame)
        acc_body = acc_data[i]
        acc_world = q_current.apply(acc_body)
        
        # C. Gravity Compensation

        # Subtract the gravity vector to get pure motion acceleration
        acc_motion = acc_world - world_gravity

        accelerations_world[i] = acc_motion
        acc_motion_mag = np.linalg.norm(acc_motion)

        raw_gyro_mag = np.linalg.norm(gyro_data[i] - gyro_bias)
        
        # D. Velocity Integration (Trapezoidal integration)
        # Define a dead zone to filter out noise when stationary and pure rotation

        # Base Threshold: 1 m/s
        # Dynamic Factor: 0.15 m/rad
        # dynamic_threshold increases with rotational speed to account for centrifugal effects
        dynamic_threshold = 1 + 0.8 * raw_gyro_mag

        global is_stationary
        global is_pure_rotation

        if acc_motion_mag < dynamic_threshold:
            acc_motion = np.zeros(3)
            v_curr = np.zeros(3)

            if raw_gyro_mag > 1:  # Rotation Threshold: 1
                is_stationary = False
        else:
            v_curr = v_curr + 0.5 * (accelerations_world[i] + accelerations_world[i-1]) * dt
            
            is_stationary = False
            is_pure_rotation = False
            
        velocities[i] = v_curr
        
        # E. Position Integration
        p_curr = p_curr + + 0.5 * (velocities[i] + velocities[i-1]) * dt
        positions[i] = p_curr

    # --- 3. Zero Velocity Update (ZUPT) ---
    # Assumption: The object returns to a static state at the end of the sequence.
    # We calculate the residual velocity error at the end and distribute it linearly backwards.
    
    v_error = velocities[-1]
    total_time = df['Time'].iloc[-1] - df['Time'].iloc[0]
    
    velocities_corrected = np.zeros_like(velocities)
    positions_corrected = np.zeros_like(positions)
    p_curr_corr = np.array([0.0, 0.0, 0.0])
    
    for i in range(1, len(df)):
        t_elapsed = df['Time'].iloc[i] - df['Time'].iloc[0]
        drift_rate = t_elapsed / total_time
        
        # Apply linear drift.
        v_corr = velocities[i] - (v_error * drift_rate)
        velocities_corrected[i] = v_corr
        
        # Re-integrate position using corrected velocity
        dt = dt_data[i]
        if dt > 1.0: dt = 0.01
        p_curr_corr = positions_corrected[i-1] + 0.5 * (velocities_corrected[i] + velocities_corrected[i-1]) * dt
        positions_corrected[i] = p_curr_corr

    # Store results back to DataFrame for plotting
    df['X'] = positions_corrected[:, 0]
    df['Y'] = positions_corrected[:, 1]
    df['Z'] = positions_corrected[:, 2]
    
    df['VX'] = velocities_corrected[:, 0]
    df['VY'] = velocities_corrected[:, 1]
    df['VZ'] = velocities_corrected[:, 2]
    
    df['AMX'] = accelerations_world[:, 0]
    df['AMY'] = accelerations_world[:, 1]
    df['AMZ'] = accelerations_world[:, 2]
    
    df['QX'] = quaternions[:, 0]
    df['QY'] = quaternions[:, 1]
    df['QZ'] = quaternions[:, 2]
    df['QW'] = quaternions[:, 3]

    # Store uncorrected position for comparison (optional)
    df['X_raw'] = positions[:, 0]
    df['Y_raw'] = positions[:, 1]
    df['Z_raw'] = positions[:, 2]

    return df

def plot_data(df):
    """
    Plots the 3D trajectory, kinematic data, and a separate ball rotation animation.
    """
    fig = plt.figure(figsize=(24, 12)) # Adjusted figure size for a 3x3 layout
    plt.suptitle("Projectile Motion Analysis (INS with ZUPT)", fontsize=16)

    # Define the grid
    grid = (3, 3)

    # --- Calculate Key Metrics ---
    vel_mag = np.linalg.norm(df[['VX', 'VY', 'VZ']].values, axis=1)
    max_speed_ms = np.max(vel_mag)
    max_speed_kmh = max_speed_ms * 3.6

    gyro_data = df[['GX', 'GY', 'GZ']].values
    gyro_mag_rads = np.linalg.norm(gyro_data, axis=1)
    max_rot_speed_rads = np.max(gyro_mag_rads)
    
    max_gyro_idx = np.argmax(gyro_mag_rads)
    max_angular_velocity_magnitude = gyro_mag_rads[max_gyro_idx]

    if max_angular_velocity_magnitude > 0:
        max_angular_velocity_axis = gyro_data[max_gyro_idx] / max_angular_velocity_magnitude
        rpm = max_angular_velocity_magnitude * 60 / (2 * np.pi)
    else:
        max_angular_velocity_magnitude = 0
        max_angular_velocity_axis = np.array([0, 0, 1]) # Default axis
        rpm = 0.0 # No rotation

    # --- Column 0: Trajectory & Velocity ---
    
    # 1. 3D Trajectory (spanning 2 rows)
    ax1 = plt.subplot2grid(grid, (0, 0), rowspan=2, projection='3d')
    ax1.plot(df['X'], df['Y'], df['Z'], label='Corrected Trajectory', linewidth=2)
    ax1.plot(df['X_raw'], df['Y_raw'], df['Z_raw'], label='Raw Integration (Drift)', linestyle=':', alpha=0.5)

    # Add start and end markers
    ax1.scatter(df['X'].iloc[0], df['Y'].iloc[0], df['Z'].iloc[0], color='green', s=100, label='Start', marker='o')
    ax1.scatter(df['X'].iloc[-1], df['Y'].iloc[-1], df['Z'].iloc[-1], color='red', s=100, label='Impact', marker='X')

    if is_stationary:
        title = "3D Trajectory (Stationary)"
    elif is_pure_rotation:
        title = "3D Trajectory (Pure Rotation)"
    else:
        title = "3D Trajectory"
    ax1.set_title(title)
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
    ax1.legend()
    ax1.set_aspect('equal', adjustable='box')

    # 2. Velocity components vs. Time
    ax2 = plt.subplot2grid(grid, (2, 0))
    ax2.plot(df['Time'], df['VX'], label='VX', alpha=0.7)
    ax2.plot(df['Time'], df['VY'], label='VY', alpha=0.7)
    ax2.plot(df['Time'], df['VZ'], label='VZ', alpha=0.7)
    ax2.plot(df['Time'], vel_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax2.set_title(f"Velocity (Max Norm: {max_speed_ms:.2f} m/s)")
    ax2.set_xlabel("Time (s)"); ax2.set_ylabel("Velocity (m/s)")
    ax2.legend(); ax2.grid(True)

    # --- Column 1: Kinematic Plots ---

    # 3. Raw Acceleration
    acc_raw_mag = np.linalg.norm(df[['AX', 'AY', 'AZ']].values, axis=1)
    max_acc_raw = np.max(acc_raw_mag)
    ax3 = plt.subplot2grid(grid, (0, 1))
    ax3.plot(df['Time'], df['AX'], label='AX', alpha=0.7)
    ax3.plot(df['Time'], df['AY'], label='AY', alpha=0.7)
    ax3.plot(df['Time'], df['AZ'], label='AZ', alpha=0.7)
    ax3.plot(df['Time'], acc_raw_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax3.set_title(f"Raw Acceleration (Max Norm: {max_acc_raw:.2f} m/s^2)")
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Accel (m/s^2)")
    ax3.legend(); ax3.grid(True)

    # 4. World Frame Acceleration
    acc_world_mag = np.linalg.norm(df[['AMX', 'AMY', 'AMZ']].values, axis=1)
    max_acc_world = np.max(acc_world_mag)
    max_gravity = np.max(gravity)
    ax4 = plt.subplot2grid(grid, (1, 1))
    ax4.plot(df['Time'], df['AMX'], label='AMX', alpha=0.7)
    ax4.plot(df['Time'], df['AMY'], label='AMY', alpha=0.7)
    ax4.plot(df['Time'], df['AMZ'], label='AMZ', alpha=0.7)
    ax4.plot(df['Time'], acc_world_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax4.set_title(f"World Frame Accel (Max Norm: {max_acc_world:.2f} m/s^2) (g = {max_gravity:.2f} m/s^2)")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Accel (m/s^2)")
    ax4.legend(); ax4.grid(True)
    
    # 5. Angular Velocity
    ax5 = plt.subplot2grid(grid, (2, 1))
    ax5.plot(df['Time'], df['GX'], label='GX', alpha=0.7)
    ax5.plot(df['Time'], df['GY'], label='GY', alpha=0.7)
    ax5.plot(df['Time'], df['GZ'], label='GZ', alpha=0.7)
    ax5.plot(df['Time'], gyro_mag_rads, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax5.set_title(f"Angular Velocity (Max Norm: {max_rot_speed_rads:.2f} rad/s)")
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Rad/s")
    ax5.legend(); ax5.grid(True)

    # --- Column 2: Summary Column ---
    
    # 6. 3D Ball Rotation Animation (spanning 2 rows)
    ax_rot = plt.subplot2grid(grid, (0, 2), rowspan=2, projection='3d')
            
    baseball_radius = 0.0368 
    animation_interval_sec = 33 / 1000.0

    u = np.linspace(0, 2 * np.pi, 12)
    v = np.linspace(0, np.pi, 7)
    x_sphere = baseball_radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = baseball_radius * np.outer(np.sin(u), np.sin(v))
    z_sphere = baseball_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    sphere_body_frame = np.stack([x_sphere.flatten(), y_sphere.flatten(), z_sphere.flatten()])
    
    info_text = f"Max Speed: {max_speed_kmh:.2f} km/h\nMax Rotation: {rpm:.2f} rpm"
    # Position text in figure coordinates. These values may need tweaking.
    fig.text(0.83, 0.5, info_text, ha='center', va='center', fontsize=14,
             bbox=dict(boxstyle="round,pad=0.5", fc="wheat", alpha=0.5))

    def update_rotation(frame):
        ax_rot.cla() 

        # Create a rotation that spins around the fixed max-velocity axis
        current_angle = max_angular_velocity_magnitude * frame * animation_interval_sec
        orientation = R.from_rotvec(max_angular_velocity_axis * current_angle)
        
        # Rotate sphere points
        sphere_world_frame = orientation.apply(sphere_body_frame.T).T
        
        x_sphere_rot = sphere_world_frame[0,:].reshape(x_sphere.shape)
        y_sphere_rot = sphere_world_frame[1,:].reshape(y_sphere.shape)
        z_sphere_rot = sphere_world_frame[2,:].reshape(z_sphere.shape)
        
        # Plot the ball's wireframe at the origin
        ax_rot.plot_wireframe(x_sphere_rot, y_sphere_rot, z_sphere_rot, color='black', lw=0.5)

        # Draw the fixed rotation axis
        axis_start = -max_angular_velocity_axis * baseball_radius * 1.5
        axis_end = max_angular_velocity_axis * baseball_radius * 1.5
        ax_rot.plot([axis_start[0], axis_end[0]], [axis_start[1], axis_end[1]], [axis_start[2], axis_end[2]], color='red', lw=2, label='Max Rotation Axis')

        ax_rot.set_title(f"Ball Rotation")
        ax_rot.set_xlabel("X"); ax_rot.set_ylabel("Y"); ax_rot.set_zlabel("Z")
        
        lim = baseball_radius * 2
        ax_rot.set_xlim(-lim, lim); ax_rot.set_ylim(-lim, lim); ax_rot.set_zlim(-lim, lim)
        ax_rot.set_aspect('equal')
        
        if frame == 0:
            ax_rot.legend(loc='upper right')
        
        return fig,

    global ani
    ani = FuncAnimation(fig, update_rotation, frames=400, blit=False, interval=33)
    
    # 7. Temperature vs. Time
    if 'Temp' in df.columns:
        ax_temp = plt.subplot2grid(grid, (2, 2))
        ax_temp.plot(df['Time'], df['Temp'], label='Temperature', color='purple')
        ax_temp.set_title("Temperature")
        ax_temp.set_xlabel("Time (s)")
        ax_temp.set_ylabel("Temp (°C)")
        ax_temp.legend()
        ax_temp.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize 3D motion data from a smart ball.')
    parser.add_argument('--input', type=str, help='Path to the input data file.')
    args = parser.parse_args()

    filepath = None
    if args.input:
        # Resolve the path from the current working directory to get an absolute path
        filepath = os.path.abspath(args.input)
    else:
        # Default file logic remains relative to the script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(script_dir, '../raw_data/projectile.txt')

    df = parse_data(filepath)
    is_stationary = True
    is_pure_rotation = True
    gravity = 9.81
    df_kinematics = calculate_kinematics(df)
    plot_data(df_kinematics)
