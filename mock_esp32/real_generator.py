import random
import time
import numpy as np

# Variables for data smoothing (Low-pass filter)
# Used to prevent the waveform from jumping abruptly and to mimic real sensor inertia
prev_ax, prev_ay, prev_az = 0, 0, -9.8
prev_gx, prev_gy, prev_gz = 0, 0, 0
alpha = 0.2  # Smoothing factor (0~1), smaller value means smoother data

PHASES = ["idle", "accelerating", "release_spin", "flight", "impact"] 
phase_index = 0
phase_start = time.time()

def generate_real_imu():
    global phase_index, phase_start
    global prev_ax, prev_ay, prev_az, prev_gx, prev_gy, prev_gz

    now = time.time()
    elapsed = now - phase_start

    phase_durations = {
        "idle": 2.0,
        "accelerating": 0.8,
        "release_spin": 0.2,
        "flight": 0.5,
        "impact": 0.05,
    }

    current_phase = PHASES[phase_index]
    if elapsed > phase_durations[current_phase]:
        phase_index = (phase_index + 1) % len(PHASES)
        phase_start = now
        current_phase = PHASES[phase_index]

    # --- Sensor Data Generation Logic ---

    if current_phase == "idle":
        target_ax = random.uniform(-0.5, 0.5)
        target_ay = random.uniform(-0.5, 0.5)
        target_az = random.uniform(-9.5, -10.1)  # Gravity
        
        target_gx = random.uniform(-2, 2)
        target_gy = random.uniform(-2, 2)
        target_gz = random.uniform(-2, 2)

    elif current_phase == "accelerating":
        # Arm Motion: Wind-up and Forward Acceleration (Ball still in hand)
        target_ax = random.uniform(15, 30)  # Strong forward acceleration
        target_ay = random.uniform(-5, 5)
        target_az = random.uniform(-5, 5)  
        
        target_gx = random.uniform(100, 300)  # Wrist rotation begins
        target_gy = random.uniform(-200, -100)
        target_gz = random.uniform(-50, 50)
        
    elif current_phase == "release_spin":
        # Release Moment: MAX Gyro and MAX Centripetal Acceleration (Ball still in contact with hand/fingers)
        # Gyro (Angular Velocity) peaks here, Accel is high due to arm force
        target_ax = random.uniform(40, 60)  # Tangential acceleration
        target_ay = random.uniform(30, 50)  # Centripetal acceleration component
        target_az = random.uniform(-20, 20)  
        
        target_gx = random.uniform(1800, 2200)  # Maximum spin rate
        target_gy = random.uniform(-100, 100)
        target_gz = random.uniform(-100, 100)
        
    elif current_phase == "flight":
        target_ax = random.uniform(-3, -1.5)
        target_ay = random.uniform(-1.5, 1.5)
        target_az = random.uniform(-1.5, 1.5)
        
        target_gx = random.uniform(1700, 2000)  # Spin rate maintained (slightly decays)
        target_gy = random.uniform(-50, 50)
        target_gz = random.uniform(-50, 50)

    elif current_phase == "impact":
        target_ax = random.uniform(-1500, -1200)
        target_ay = random.uniform(-100, 100)
        target_az = random.uniform(-100, 100)  # Massive impact force
        
        target_gx = random.uniform(-10, 10)  # Spin is instantly stopped
        target_gy = random.uniform(-10, 10)
        target_gz = random.uniform(-10, 10)
        
    else:
        target_ax = target_ay = target_az = target_gx = target_gy = target_gz = 0.0

    # --- Apply Smoothing (Exponential Moving Average) ---
    
    ax = alpha * target_ax + (1 - alpha) * prev_ax
    ay = alpha * target_ay + (1 - alpha) * prev_ay
    az = alpha * target_az + (1 - alpha) * prev_az
    
    gx = alpha * target_gx + (1 - alpha) * prev_gx
    gy = alpha * target_gy + (1 - alpha) * prev_gy
    gz = alpha * target_gz + (1 - alpha) * prev_gz

    # Update historical values
    prev_ax, prev_ay, prev_az = ax, ay, az
    prev_gx, prev_gy, prev_gz = gx, gy, gz
    
    t = 29.0 + random.uniform(0, 1.0) + phase_index * 0.2
    return f"AX:{ax:.2f}, AY:{ay:.2f}, AZ:{az:.2f}, GX:{gx:.2f}, GY:{gy:.2f}, GZ:{gz:.2f}, T:{t:.2f}, PHASE:{current_phase}"