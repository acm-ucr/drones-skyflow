import RPi.GPIO as GPIO
import numpy as np
import math
import time
import mpu6050
import time



# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, dt):
        self.dt = dt
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
# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    # Read temp
    temperature = mpu6050.get_temp()

    return accelerometer_data, gyroscope_data, temperature

def set_speed(speed, motor):
    #Speed should be a value between 0 and 100 
    duty_cycle = 2.5 + (speed / 100) * 10  
    motor.ChangeDutyCycle(duty_cycle)
    print(f"Speed set to {speed}%")
    #example usage:
    #up_right_speed = 10
    #set_speed(up_right_speed, motor_up_right)


MOTOR_1=15
MOTOR_2=7
MOTOR_3=15
MOTOR_4=20
pulsewidth_1=1000
pulsewidth_2=1000
pulsewidth_3=1000
pulsewidth_4=1000



# PID Gains (Tune these - might need different gains for roll/pitch/yaw if dynamics differ)
Kp_roll = 24
Ki_roll = 20
Kd_roll = 10

Kp_pitch = 12.0
Ki_pitch = 2.5
Kd_pitch = 1.8

Kp_yaw = 5.0
Ki_yaw = 0.5
Kd_yaw = 0.8

Kp_alt = 4.0
Ki_alt = 0.2
Kd_alt = 2.5
MAX_THRUST_CORRECTION = 10.0 # Max thrust correction (Newtons) for altitude PID

# System Parameters
setpoint_roll = 0.0
setpoint_pitch = 0.0
setpoint_yaw = 0.0
setpoint_altitude = 5.0 # Target altitude (meters)
dt = 0.01
initial_roll = 0
initial_pitch = 0
initial_yaw = 0
initial_altitude = 0

# --- Initialize PIDs ---
#NOTE: these are the amounts to be sent to the motor in some way shape or form
pid_roll = PIDController(Kp_roll, Ki_roll, Kd_roll, setpoint_roll, dt)
pid_pitch = PIDController(Kp_pitch, Ki_pitch, Kd_pitch, setpoint_pitch, dt)
pid_yaw = PIDController(Kp_yaw, Ki_yaw, Kd_yaw, setpoint_yaw, dt)
pid_altitude = PIDController(Kp_alt, Ki_alt, Kd_alt, setpoint_altitude, dt)

complementary_filter_alpha = 0.98 # Initial alpha value (tune this)
estimated_roll = 0.0
estimated_pitch = 0.0


# --- Initial State ---
current_roll = 0
current_pitch = 0
current_yaw = 0
current_altitude = 0 # Initialize current altitude
current_vertical_velocity = 0 # Initialize vertical velocity



# initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon. Exiting.")
    exit()

# send minimum throttle signal to the ESC to initialize
pi.set_servo_pulsewidth(MOTOR_1, 1000)  # 1000µs is usually the ESC idle
time.sleep(2)

print("Motor 1 starting...")

# send minimum throttle signal to the ESC to initialize
pi.set_servo_pulsewidth(MOTOR_2, 1000)  # 1000µs is usually the ESC idle
time.sleep(2)

print("Motor 2 starting...")

pi.set_servo_pulsewidth(MOTOR_3, 1000)  # 1000µs is usually the ESC idle
time.sleep(2)

print("Motor 3 starting...")

pi.set_servo_pulsewidth(MOTOR_4, 1000)  # 1000µs is usually the ESC idle
time.sleep(2)

print("Motor 4 starting...")


DroneRunning = True
start_time = time.time()
complementaryPitch = 0.0
complementaryRoll = 0.0
estimated_yaw = 0.0
#we assume that 3000 in all motors is hovering

while (DroneRunning): 
    dt = time.time() - start_time
    #hover code
    gyro_data, accel_data, temperature = read_sensor_data
    gyro_data = np.degrees(gyro_data) # Convert gyro to deg/s

    try:
        pitchFromAccel = math.atan2(-accel_data['x'] / math.sqrt(math.pow(accel_data['y'], 2) + math.pow(accel_data['z'], 2)))
        rollFromAccel = math.atan2(accel_data['y'] / math.sqrt(math.pow(accel_data['x'], 2) + math.pow(accel_data['z'], 2)))
    except (ValueError, ZeroDivisionError):
        pitchFromAccel = complementaryPitch
        rollFromAccel = complementaryRoll

    comp_filter_gain = 0.98
    complementaryPitch = comp_filter_gain * (complementaryPitch + (gyro_data['y'] * dt)) + (1 - comp_filter_gain) * pitchFromAccel
    complementaryRoll = comp_filter_gain * (complementaryRoll + (gyro_data['x'] * dt)) + (1 - comp_filter_gain) * rollFromAccel
    
    estimated_yaw += gyro_data['z'] * dt
    #how much we need to adjust by, rn are just numbers
    roll_torque = pid_roll.update(complementaryRoll, dt)
    pitch_torque = pid_pitch.update(complementaryPitch, dt)

    #we can figure out yaw seperatley
    yaw_torque = pid_yaw.update(estimated_yaw, dt)


    #idea: each motor has 1000 base, then add some arbitary number scaling with each of the torques to the motors to roll/pitchh/yaw them
    MOTOR_BASE = 1000
    #example, idk which direction pitch, etc is until we get the readings
    #probably very naive; however, the torques are calculated with PID so
    #is some kind of trig to deal w/ the current angle of the drone neccessary? 
    #eh, assume not for now
    motor1speed = MOTOR_BASE + pitch_torque + roll_torque + yaw_torque
    motor2speed = MOTOR_BASE + pitch_torque + -1 * roll_torque + -1 * yaw_torque
    motor3speed = MOTOR_BASE + -1 * pitch_torque + roll_torque + -1 * yaw_torque
    motor4speed = MOTOR_BASE + -1 * pitch_torque + -1 * + roll_torque + yaw_torque

    motor1speed = max(1000, motor1speed)
    motor2speed = max(1000, motor1speed)
    motor3speed = max(1000, motor1speed)
    motor4speed = max(1000, motor1speed)

    pi.set_servo_pulsewidth(MOTOR_1, motor1speed) 
    pi.set_servo_pulsewidth(MOTOR_2, motor2speed)
    pi.set_servo_pulsewidth(MOTOR_3, motor3speed)
    pi.set_servo_pulsewidth(MOTOR_4, motor4speed)


#camrea detection ideas
#to follow a track of cones, rotate such that the two largest cones are equidistant from the center, but also in proportion to their relative sizes
#rotateo more towards the smaller cone
#go forward
#maybe also, 

# FRONT
#  1   2 
#   \ /
#    |
#   / \
#  3   4
# BACK
#pitch forward is  forward
#roll is

    # Altitude PID output is thrust correction.
    #base_thrust = 3000 # Thrust needed to hover
    
    #altitude_thrust_correction = pid_altitude.update(current_altitude)
    # Clamp the PID correction output to prevent excessive thrust
    #altitude_thrust_correction = max(-MAX_THRUST_CORRECTION, min(MAX_THRUST_CORRECTION, altitude_thrust_correction))
    #total_thrust = base_thrust + altitude_thrust_correction
    # Ensure total thrust isn't negative (though physics update also handles this)
    #total_thrust = max(0, total_thrust)
    