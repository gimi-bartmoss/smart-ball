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
    
    # Convert timestamp from milliseconds to seconds
    df['Time'] = df['Timestamp'] / 1000.0
    
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
    # The goal is to align the sensor's measured gravity vector with the World frame gravity [0, 0, -g].
    n_static = 20
    if len(df) < n_static:
        raise ValueError("Not enough data points for initial alignment.")

    acc_static = df[['AX', 'AY', 'AZ']].iloc[:n_static].mean().values
    
    # Define World Gravity Vector (Assuming Z-up coordinate system, gravity points down)
    world_gravity = np.array([0, 0, -9.81])
    
    # Normalize vectors for calculation
    acc_static_norm = acc_static / np.linalg.norm(acc_static)
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
        omega = gyro_data[i]
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
        
        # D. Velocity Integration (Euler method)
        v_curr = v_curr + acc_motion * dt
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
    
    # Store uncorrected position for comparison (optional)
    df['X_raw'] = positions[:, 0]
    df['Y_raw'] = positions[:, 1]
    df['Z_raw'] = positions[:, 2]

    return df

def plot_data(df):
    """
    Plots the 3D trajectory and kinematic data.
    """
    fig = plt.figure(figsize=(15, 12))
    plt.suptitle("Projectile Motion Analysis (INS with ZUPT)", fontsize=16)

    # 1. 3D Trajectory
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.plot(df['X'], df['Y'], df['Z'], label='Corrected Trajectory', linewidth=2)
    # Optional: Plot raw trajectory to show drift magnitude
    ax1.plot(df['X_raw'], df['Y_raw'], df['Z_raw'], label='Raw Integration (Drift)', linestyle=':', alpha=0.5)
    ax1.set_title("3D Trajectory")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_zlabel("Z (m)")
    ax1.legend()

    # 2. Velocity components vs. Time
    # Calculate Max Speed (Norm of velocity vector)
    vel_mag = np.linalg.norm(df[['VX', 'VY', 'VZ']].values, axis=1)
    max_speed = np.max(vel_mag)

    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(df['Time'], df['VX'], label='VX', alpha=0.7)
    ax2.plot(df['Time'], df['VY'], label='VY', alpha=0.7)
    ax2.plot(df['Time'], df['VZ'], label='VZ', alpha=0.7)
    ax2.plot(df['Time'], vel_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    # Display Max Speed in Title
    ax2.set_title(f"Velocity (Max Norm: {max_speed:.2f} m/s)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.legend()
    ax2.grid(True)

    # 3. Acceleration (Three Axis + Norm)
    acc_mag = np.linalg.norm(df[['AX', 'AY', 'AZ']].values, axis=1)
    max_acc = np.max(acc_mag)

    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(df['Time'], df['AX'], label='AX', alpha=0.7)
    ax3.plot(df['Time'], df['AY'], label='AY', alpha=0.7)
    ax3.plot(df['Time'], df['AZ'], label='AZ', alpha=0.7)
    ax3.plot(df['Time'], acc_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    ax3.set_title(f"Acceleration (Max Norm: {max_acc:.2f} m/s^2)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Accel (m/s^2)")
    ax3.legend()
    ax3.grid(True)
    
    # 4. Angular Velocity (Three Axis + Norm)
    # Calculate Max Rotation Speed
    gyro_mag = np.linalg.norm(df[['GX', 'GY', 'GZ']].values, axis=1)
    max_rot_speed = np.max(gyro_mag)

    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(df['Time'], df['GX'], label='GX', alpha=0.7)
    ax4.plot(df['Time'], df['GY'], label='GY', alpha=0.7)
    ax4.plot(df['Time'], df['GZ'], label='GZ', alpha=0.7)
    ax4.plot(df['Time'], gyro_mag, label='Norm', color='black', linestyle='--', linewidth=1.5)
    # Display Max Rotation Speed in Title
    ax4.set_title(f"Angular Velocity (Max Norm: {max_rot_speed:.2f} rad/s)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Rad/s")
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filepath = os.path.join(script_dir, 'projectile_1.txt')
            
    df = parse_data(filepath)
    df_kinematics = calculate_kinematics(df)
    plot_data(df_kinematics)