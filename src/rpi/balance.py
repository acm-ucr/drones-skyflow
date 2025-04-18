#assume that we already have data coming in from the acceleratmeters
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
import numpy as np
import math
import time


#accel -x is forward
#accel +y is right
#accel +z is up

#+x is roll left
#+y is pitch back
#+z is yaw left 
ESC_PIN = 0  
sensor = mpu6050(0x68) # I2C address of MPU6050 is 0x68, can be different
command = "direction"

motor_up_left = GPIO.PWM(ESC_PIN, 50)  
motor_up_right = GPIO.PWM(ESC_PIN, 50)
motor_down_left = GPIO.PWM(ESC_PIN, 50)
motor_down_right = GPIO.PWM(ESC_PIN, 50)


def set_speed(speed, motor):
    #Speed should be a value between 0 and 100 
    duty_cycle = 2.5 + (speed / 100) * 10  
    motor.ChangeDutyCycle(duty_cycle)
    print(f"Speed set to {speed}%")
    #example usage:
    #up_right_speed = 10
    #set_speed(up_right_speed, motor_up_right)



# --- Constants ---
YAW_DAMPING_FACTOR = 0.1 # Specific damping for yaw, can be tuned
GRAVITY = 9.81
VERTICAL_DAMPING_FACTOR = 0.2 # Damping for vertical motionpip

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
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
initial_roll = .5
initial_pitch = .5
initial_yaw = 0
initial_altitude = setpoint_altitude # Start AT setpoint altitude
platform_mass = 1.0 # Kilograms
disturbance_magnitude = 500.0 # Torque for roll/pitch
yaw_disturbance_magnitude = 50.0 # Torque for yaw
vertical_disturbance_force = 150.0 # Increased Force for altitude disturbance (Newtons)

# --- Initialize PIDs ---
pid_roll = PIDController(Kp_roll, Ki_roll, Kd_roll, setpoint_roll, dt)
pid_pitch = PIDController(Kp_pitch, Ki_pitch, Kd_pitch, setpoint_pitch, dt)
pid_yaw = PIDController(Kp_yaw, Ki_yaw, Kd_yaw, setpoint_yaw, dt)
pid_altitude = PIDController(Kp_alt, Ki_alt, Kd_alt, setpoint_altitude, dt)
#platform = BalancingPlatform(initial_roll=initial_roll, initial_pitch=initial_pitch, initial_yaw=initial_yaw,
#                            initial_altitude=initial_altitude, mass=platform_mass)

complementary_filter_alpha = 0.98 # Initial alpha value (tune this)
estimated_roll = 0.0
estimated_pitch = 0.0


# --- Initial State ---
current_roll = 0
current_pitch = 0
current_yaw = 0
current_altitude = 0 # Initialize current altitude
current_vertical_velocity = 0 # Initialize vertical velocity


gyro_rad, accel_ms2 = platform.get_imu_readings()
gyro_deg = np.degrees(gyro_rad) # Convert gyro to deg/s for display

# --- Complementary Filter Calculation ---
# Estimate roll/pitch from accelerometer
acc_x, acc_y, acc_z = accel_ms2
# Avoid division by zero if acc_z is near zero
# Also handle potential domain errors for asin if acc_y is outside [-|acc_z|, |acc_z|] due to noise/sim issues
try:
    # Roll based on Y and Z components
    accel_roll = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) # More robust atan2 version
    # Pitch based on X and Z components
    accel_pitch = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) # More robust atan2 version
    # accel_roll = math.atan2(acc_y, acc_z) # Simpler version, prone to issues near +/- 90 deg pitch
    # accel_pitch = math.asin(-acc_x / GRAVITY) # Simpler version, assumes gravity magnitude is constant and known
except (ValueError, ZeroDivisionError):
    accel_roll = estimated_roll
    accel_pitch = estimated_pitch

# Gyro integration (simple Euler integration)
gyro_x_rad, gyro_y_rad, _ = gyro_rad
gyro_roll_delta = gyro_x_rad * dt
gyro_pitch_delta = gyro_y_rad * dt

# Apply complementary filter
# angle = alpha * (angle + gyro * dt) + (1 - alpha) * (accel_angle)
estimated_roll = complementary_filter_alpha * (estimated_roll + gyro_roll_delta) + \
                    (1 - complementary_filter_alpha) * accel_roll
estimated_pitch = complementary_filter_alpha * (estimated_pitch + gyro_pitch_delta) + \
                    (1 - complementary_filter_alpha) * accel_pitch






























up_left_speed = 0
up_right_speed = 0
down_left_speed = 0
down_right_speed = 0
#x is left/right, y, z
averaged_acc = [0 , 0 , 0]
averaged_gyro = [0,0,0]

prev_acc = [[0,0,0],[0,0,0]]
prev_gyro = [[0,0,0],[0,0,0]]

#+roll is roll left
#+pitch is pitch back
#+yaw is yaw left
pitch = 0
roll = 0
yaw = 0
while True:

    #get cleaned readings
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    
    #'clean' the data by averaging over last 3 readings
    for i in range(3):
        averaged_acc[i] = (prev_acc[0][i] + prev_acc[1][i]+accel_data[i])/3
        averaged_gyro[i] = (prev_gyro[0][i] + prev_gyro[1][i]+gyro_data[i])/3

    #TODO:: Calculator roll, pitch, yaw

    #hovering
    #rolling left
    if roll > 1:
        up_left_speed += 1
        down_left_speed += 1
        up_right_speed -= 1
        down_right_speed -= 1
    elif roll < -1:
        up_left_speed -= 1
        down_left_speed -= 1
        up_right_speed += 1
        down_right_speed += 1

    if pitch > 1:
        up_left_speed -= 1
        down_left_speed += 1
        up_right_speed -= 1
        down_right_speed += 1
    elif pitch < -1:
        up_left_speed += 1
        down_left_speed -= 1
        up_right_speed += 1
        down_right_speed -= 1
    
    #figure out where we want to be
    if command == "HOVER":
        potato = 1
        #do nothing type
    elif command == "LEFT":
        piotaot = 2
        #go elft
    elif command == "RIGHT":
        #go right
        potao = 2
    
    if (averaged_acc[1] >)
    #idea one: to handle spikes in data (if messy), use some kind of rolling average/median?


    #move 

    #moves data
    prev_acc[0] = prev_acc[1]
    prev_gyro[0] = prev_gyro[1]

    prev_acc[1] = accel_data
    prev_gyro[1] = gyro_data

#relevant variables: acceldata['x'], y, z; gyro_data['x'], y, z

