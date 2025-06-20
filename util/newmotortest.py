import time
import threading
import board
import busio
import signal
import sys
from adafruit_pca9685 import PCA9685

import mpu6050
import math
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = time.time()
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value):
        error = self.setpoint - current_value
        self.integral += error * self.dt
        # Avoid division by zero if dt is very small or zero
        if self.dt > 1e-9:
            derivative = (error - self.previous_error) / self.dt
        else:
            derivative = 0.0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

    def reset(self):
        self.integral = 0
        self.previous_error = 0

Kp = 25
Ki = 15
Kd = 5
setpoint_roll = 0.0
setpoint_pitch = 0.0
setpoint_yaw = 0.0
piddt = 0.001
pid_roll = PIDController(Kp, Ki, Kd, setpoint_roll, piddt)
pid_pitch = PIDController(Kp, Ki, Kd, setpoint_pitch, piddt)
pid_yaw = PIDController(Kp, Ki, Kd, setpoint_yaw, piddt) #1/50th of a second

# Initialize MPU6050
mpu = mpu6050.mpu6050(0x68)

# Complementary filter state
complementaryPitch = 0.0
complementaryRoll = 0.0
comp_filter_gain = 0.98
last_time = time.time()

def read_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    return accel, gyro

def get_balanced_throttles(base_throttle=1000):
    global roll_torque, pitch_torque, yaw_torque, accel_force
    global complementaryPitch, complementaryRoll, last_time

    # Time delta
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Read sensors
    accel_data, gyro_data = read_sensor_data()

    ## Compute angles from accelerometer
    try:
        pitch_acc = math.atan2(accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2))
        roll_acc = math.atan2(accel_data['y'], accel_data['z'])
    except ZeroDivisionError:
        pitch_acc = estimated_roll
        roll_acc = estimated_pitch


    ## Gyro rates in deg/s
    gyro_pitch_rate = gyro_data['y']
    gyro_roll_rate = gyro_data['x']

    # Apply complementary filter
    complementaryPitch = comp_filter_gain * (complementaryPitch + math.radians(gyro_pitch_rate) * dt) + (1 - comp_filter_gain) * pitch_acc
    complementaryRoll = comp_filter_gain * (complementaryRoll + math.radians(gyro_roll_rate) * dt) + (1 - comp_filter_gain) * roll_acc


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

    print(f"Pitch: {pitch_deg:.2f}° ({pitch_msg}), Roll: {roll_deg:.2f}° ({roll_msg})")

    


    # Proportional gains (tune these!)
    pitch_kp = 1.2
    roll_kp = 1.2

    pitch_torque = pid_roll.update(complementaryPitch)
    roll_torque = pid_roll.update(complementaryRoll)
    # Compute pitch and roll adjustments
    pitch_adjust = pitch_kp * pitch_deg
    roll_adjust = roll_kp * roll_deg

    # Motor mapping:
    # M0: Front Right
    # M1: Back Right
    # M2: Back Left
    # M3: Front Left

    #return {
    #    0: base_throttle - pitch_adjust + roll_adjust,  # Front Right
    #    1: base_throttle + pitch_adjust + roll_adjust,  # Back Right
    #    2: base_throttle + pitch_adjust - roll_adjust,  # Back Left
    #    3: base_throttle - pitch_adjust - roll_adjust   # Front Left
    #}
    return (
        base_throttle + pitch_torque ,  # Front Right
        base_throttle - pitch_torque ,  # Back Right
        base_throttle - pitch_torque ,  # Back Left
        base_throttle + pitch_torque   # Front Left
    )


# Setup I2C
i2c = busio.I2C(board.SCL, board.SDA)
while not i2c.try_lock():
    pass
i2c.unlock()

# Setup PCA9685
pca = PCA9685(i2c)
pca.frequency = 50  # Standard for ESCs
time.sleep(1)

# Convert microseconds to 16-bit duty cycle
def pulse_us_to_duty(pulse_us):
    pulse_length = 1000000 / pca.frequency
    return int((pulse_us / pulse_length) * 0xFFFF)

# Emergency stop on Ctrl+C
def emergency_stop(signal_received, frame):
    print("\n[!] Emergency Stop Triggered! Setting all motors to 1000µs...")
    for ch in motor_channels:
        pca.channels[ch].duty_cycle = pulse_us_to_duty(1000)
    time.sleep(1)
    pca.deinit()
    print("PCA9685 deinitialized. Exiting.")
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)

def print_overview():
    try:
        while True:
            print("\n" + "="*30)
            print("         SkyFlow Overview")
            print("="*30)
            print(f"          Front")
            print()
            print(f"M3: {pca.channels[3].duty_cycle}µs     M0: {pca.channels[0].duty_cycle}µs")
            print("         \\   /")
            print("          \\ /")
            print("           |")
            print("          / \\")
            print("         /   \\")
            print(f"M2: {pca.channels[2].duty_cycle}µs     M1: {pca.channels[1].duty_cycle}µs")
            print("          Back")
            print("="*30 + "\n")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user.")

#Insert PID function here!!!!!

try:
    # Define motor pins (channels) and their initial speeds and max speeds
    motor_channels = [0, 1, 2, 3]  # Assigning motors to channels
    thread = threading.Thread(target=print_overview)
    thread.daemon = True  # dies when main program exits
    thread.start()
    #motor_speeds = {
    #    0: {"current": 1000, "max": 1100},  # Motor 1: Starts at 1000µs, max at 1400µs
    #    1: {"current": 1000, "max": 1100},  # Motor 2: Starts at 1000µs, max at 1500µs
    #    2: {"current": 1000, "max": 1100},  # Motor 3: Starts at 1000µs, max at 1600µs
    #    3: {"current": 1000, "max": 1100}  # Motor 4: Starts at 1000µs, max at 1700µs
    #}
    motor_speeds = (
        1000,  # Motor 1: Starts at 1000µs, max at 1400µs
        1000,  # Motor 2: Starts at 1000µs, max at 1500µs
        1000,  # Motor 3: Starts at 1000µs, max at 1600µs
        1000  # Motor 4: Starts at 1000µs, max at 1700µs
    )

    # Arm all motors at their respective starting speeds
    print("Arming all ESCs at their starting speeds...")
    for motor_channel in motor_channels:
        pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(motor_speeds[motor_channel])
    time.sleep(5)

    print("[INFO] Starting real-time balancing loop...")
    base_throttle = 1000
    # originally 1600

    start_time = time.time()
    duration = 60  # seconds

    while time.time() - start_time < duration:
        motor_throttles = get_balanced_throttles(base_throttle)

        n = 0
        for throttle in motor_throttles:
            throttle = min(1200, max(1000, int(throttle)))  # Clamp for safety and spin-up
            pca.channels[n].duty_cycle = pulse_us_to_duty(throttle)
            n = n+1

        time.sleep(0.001)  # 20 Hz update rate




    # pca.channels[0].duty_cycle = pulse_us_to_duty(1600)
    # pca.channels[1].duty_cycle = pulse_us_to_duty(1600)
    # pca.channels[2].duty_cycle = pulse_us_to_duty(1600)
    # pca.channels[3].duty_cycle = pulse_us_to_duty(1600)

    #time.sleep(15)

    pca.channels[0].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[1].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[2].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[3].duty_cycle = pulse_us_to_duty(1100)

    time.sleep(0.5)

    pca.channels[0].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[1].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[2].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[3].duty_cycle = pulse_us_to_duty(1000)


    print("Increasing throttle on all motors...")
    print("Decreasing throttle on all motors...")

    print("Stopping all motors.")
    print("\nAll motors tested successfully.")

finally:
    # Final cleanup if not already stopped
    for motor_channel in motor_channels:
        pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(1000)
    pca.deinit()
    print("PCA9685 deinitialized.")