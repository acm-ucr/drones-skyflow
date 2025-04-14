#assume that we already have data coming in from the acceleratmeters
from mpu6050 import mpu6050
import RPi.GPIO as GPIO

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

up_left_speed = 0
up_right_speed = 0
down_left_speed = 0
down_right_speed = 0

def set_speed(speed, motor):
    #Speed should be a value between 0 and 100 
    duty_cycle = 2.5 + (speed / 100) * 10  
    motor.ChangeDutyCycle(duty_cycle)
    print(f"Speed set to {speed}%")
    #example usage:
    #up_right_speed = 10
    #set_speed(up_right_speed, motor_up_right)

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
        down_right_speed -= 1
    elif 
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

