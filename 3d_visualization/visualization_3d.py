import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

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
    
    # State vectors
    v_curr = np.array([0.0, 0.0, 0.0])
    p_curr = np.array([0.0, 0.0, 0.0])
    
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
        
        # B. Specific Force Projection (Body Frame -> World Frame)
        acc_body = acc_data[i]
        acc_world = q_current.apply(acc_body)
        
        # C. Gravity Compensation

        # Subtract the gravity vector to get pure motion acceleration
        acc_motion = acc_world - world_gravity

        accelerations_world[i] = acc_motion
        acc_motion_mag = np.linalg.norm(acc_motion)

        raw_gyro_mag = np.linalg.norm(gyro_data[i] - gyro_bias)
        
        # D. Velocity Integration (Euler method)
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
            v_curr = v_curr + acc_motion * dt
            
            is_stationary = False
            is_pure_rotation = False
            
        velocities[i] = v_curr
        
        # E. Position Integration
        p_curr = p_curr + v_curr * dt
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
        
        # Apply linear drift correction to velocity
        v_corr = velocities[i] - (v_error * drift_rate)
        velocities_corrected[i] = v_corr
        
        # Re-integrate position using corrected velocity
        dt = dt_data[i]
        if dt > 1.0: dt = 0.01
        p_curr_corr = p_curr_corr + v_corr * dt
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
    
    # Store uncorrected position for comparison (optional)
    df['X_raw'] = positions[:, 0]
    df['Y_raw'] = positions[:, 1]
    df['Z_raw'] = positions[:, 2]

    return df

def plot_data(df):
    """
    Plots the 3D trajectory and kinematic data.
    """
    fig = plt.figure(figsize=(18, 12)) # Adjusted figure size for a better layout
    plt.suptitle("Projectile Motion Analysis (INS with ZUPT)", fontsize=16)

    # Define the grid
    grid = (3, 2)

    # 1. 3D Trajectory (Left column, spanning 2 rows)
    ax1 = plt.subplot2grid(grid, (0, 0), rowspan=2, projection='3d')
    ax1.plot(df['X'], df['Y'], df['Z'], label='Corrected Trajectory', linewidth=2)
    ax1.plot(df['X_raw'], df['Y_raw'], df['Z_raw'], label='Raw Integration (Drift)', linestyle=':', alpha=0.5)

    if is_stationary:
        title = "3D Trajectory (Stationary)"
    elif is_pure_rotation:
        title = "3D Trajectory (Pure Rotation)"
    else:
        title = "3D Trajectory"
    ax1.set_title(title)

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.legend()

    # 2. Velocity components vs. Time (Left column, bottom)
    vel_mag = np.linalg.norm(df[['VX', 'VY', 'VZ']].values, axis=1)
    max_speed = np.max(vel_mag)
    ax2 = plt.subplot2grid(grid, (2, 0))
    ax2.plot(df['Time'], df['VX'], label='VX', alpha=0.7)
    ax2.plot(df['Time'], df['VY'], label='VY', alpha=0.7)
    ax2.plot(df['Time'], df['VZ'], label='VZ', alpha=0.7)
    ax2.plot(df['Time'], vel_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax2.set_title(f"Velocity (Max Norm: {max_speed:.2f} m/s)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.legend()
    ax2.grid(True)

    # 3. Raw Acceleration (Right column, top)
    acc_raw_mag = np.linalg.norm(df[['AX', 'AY', 'AZ']].values, axis=1)
    max_acc_raw = np.max(acc_raw_mag)
    ax3 = plt.subplot2grid(grid, (0, 1))
    ax3.plot(df['Time'], df['AX'], label='AX', alpha=0.7)
    ax3.plot(df['Time'], df['AY'], label='AY', alpha=0.7)
    ax3.plot(df['Time'], df['AZ'], label='AZ', alpha=0.7)
    ax3.plot(df['Time'], acc_raw_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax3.set_title(f"Raw Acceleration (Max Norm: {max_acc_raw:.2f} m/s^2)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Accel (m/s^2)")
    ax3.legend()
    ax3.grid(True)

    # 4. World Frame Acceleration (Right column, middle)
    acc_world_mag = np.linalg.norm(df[['AMX', 'AMY', 'AMZ']].values, axis=1)
    max_acc_world = np.max(acc_world_mag)
    ax4 = plt.subplot2grid(grid, (1, 1))
    ax4.plot(df['Time'], df['AMX'], label='AMX', alpha=0.7)
    ax4.plot(df['Time'], df['AMY'], label='AMY', alpha=0.7)
    ax4.plot(df['Time'], df['AMZ'], label='AMZ', alpha=0.7)
    ax4.plot(df['Time'], acc_world_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax4.set_title(f"World Frame Accel (Max Norm: {max_acc_world:.2f} m/s^2)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Accel (m/s^2)")
    ax4.legend()
    ax4.grid(True)
    
    # 5. Angular Velocity (Right column, bottom)
    gyro_mag = np.linalg.norm(df[['GX', 'GY', 'GZ']].values, axis=1)
    max_rot_speed = np.max(gyro_mag)
    ax5 = plt.subplot2grid(grid, (2, 1))
    ax5.plot(df['Time'], df['GX'], label='GX', alpha=0.7)
    ax5.plot(df['Time'], df['GY'], label='GY', alpha=0.7)
    ax5.plot(df['Time'], df['GZ'], label='GZ', alpha=0.7)
    ax5.plot(df['Time'], gyro_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax5.set_title(f"Angular Velocity (Max Norm: {max_rot_speed:.2f} rad/s)")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Rad/s")
    ax5.legend()
    ax5.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, '../raw_data/projectile.txt')
            
    df = parse_data(filepath)
    is_stationary = True
    is_pure_rotation = True
    df_kinematics = calculate_kinematics(df)
    plot_data(df_kinematics)