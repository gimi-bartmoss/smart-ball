import random

# Function to generate random mock IMU data
def generate_random_imu():
    ax = random.uniform(-5, 15)
    ay = random.uniform(-3, 3)
    az = random.uniform(-5, 5)
    gx = random.uniform(-5, 5)
    gy = random.uniform(-30, 0)
    gz = random.uniform(-5, 5)
    t  = random.uniform(28.0, 33.0)

    return f"AX:{ax:.2f}, AY:{ay:.2f}, AZ:{az:.2f}, GX:{gx:.2f}, GY:{gy:.2f}, GZ:{gz:.2f}, T:{t:.2f}"
