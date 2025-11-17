import random
import time

PHASES = ["idle", "accelerating", "spinning", "stopping"]
phase_index = 0
phase_start = time.time()

def generate_real_imu():
    global phase_index, phase_start
    now = time.time()
    elapsed = now - phase_start

    phase_durations = {
        "idle": 2.0,
        "accelerating": 1.0,
        "spinning": 2.0,
        "stopping": 1.0,
    }

    current_phase = PHASES[phase_index]
    if elapsed > phase_durations[current_phase]:
        phase_index = (phase_index + 1) % len(PHASES)
        phase_start = now
        current_phase = PHASES[phase_index]

    if current_phase == "idle":
        ax = random.uniform(-0.2, 0.2)
        ay = random.uniform(-0.2, 0.2)
        az = random.uniform(9.5, 9.8)
        gx = gy = gz = random.uniform(-0.5, 0.5)
    elif current_phase == "accelerating":
        ax = random.uniform(10, 25)
        ay = random.uniform(-5, 5)
        az = random.uniform(-5, 15)
        gx = random.uniform(50, 150)
        gy = random.uniform(-50, 50)
        gz = random.uniform(-30, 30)
    elif current_phase == "spinning":
        ax = random.uniform(-5, 5)
        ay = random.uniform(-5, 5)
        az = random.uniform(-2, 2)
        gx = random.uniform(500, 2000)
        gy = random.uniform(-500, 500)
        gz = random.uniform(-500, 500)
    elif current_phase == "stopping":
        ax = random.uniform(-3, 3)
        ay = random.uniform(-3, 3)
        az = random.uniform(8.0, 9.8)
        gx = gy = gz = random.uniform(-10, 10)
    else:
        ax = ay = az = gx = gy = gz = 0.0

    t = 29.0 + random.uniform(0, 1.0) + phase_index * 0.2
    return f"AX:{ax:.2f}, AY:{ay:.2f}, AZ:{az:.2f}, GX:{gx:.2f}, GY:{gy:.2f}, GZ:{gz:.2f}, T:{t:.2f}, PHASE:{current_phase}"
