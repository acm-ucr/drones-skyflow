import mpu6050
import math
import time

# create mpu6050 object
mpu = mpu6050.mpu6050(0x68)

# Initialize complementary filter values
complementaryPitch = 0.0
complementaryRoll = 0.0
comp_filter_gain = 0.98

# Initial time
last_time = time.time()

def read_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    return accel, gyro

while True:
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    accel_data, gyro_data = read_sensor_data()

    # Accelerometer angles
    try:
        pitch_acc = math.atan2(accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2))
        roll_acc = math.atan2(accel_data['y'], accel_data['z'])
    except ZeroDivisionError:
        continue

    # Gyroscope integration (deg/s)
    gyro_pitch_rate = gyro_data['y']
    gyro_roll_rate = gyro_data['x']

    # Update complementary filter (convert gyro from deg/s to rad/s)
    complementaryPitch = comp_filter_gain * (complementaryPitch + math.radians(gyro_pitch_rate) * dt) + (1 - comp_filter_gain) * pitch_acc
    complementaryRoll = comp_filter_gain * (complementaryRoll + math.radians(gyro_roll_rate) * dt) + (1 - comp_filter_gain) * roll_acc

    # Convert to degrees
    pitch_deg = math.degrees(complementaryPitch)
    roll_deg = math.degrees(complementaryRoll)

    # Orientation message
    pitch_msg = "Level"
    if pitch_deg > 5:
        pitch_msg = "Tilting Backward"
    elif pitch_deg < -5:
        pitch_msg = "Tilting Forward"

    roll_msg = "Level"
    if roll_deg > 5:
        roll_msg = "Tilting Right"
    elif roll_deg < -5:
        roll_msg = "Tilting Left"

    # Output
    print(f"Pitch: {pitch_deg:.2f}° ({pitch_msg})")
    print(f"Roll: {roll_deg:.2f}° ({roll_msg})")
    print("-" * 40)

    time.sleep(0.001)  # 1000 Hz update rate
