# cli util
  # sudo apt install python3-smbus
  # pip install mpu6050-raspberrypi

import mpu6050
import math
import time

# create mpu6050 objec  t
mpu6050 = mpu6050.mpu6050(0x68)

# get accel and gyro data
def read_sensor_data():
  accel_data = mpu6050.get_accel_data()
  gyro_data = mpu6050.get_gyro_data()
  
  return accel_data, gyro_data

while True:
  start_time = time.time()
  
  # g and deg/s
  accel_data, gyro_data = read_sensor_data()
  
  print(f"accel_data: {accel_data}, gyro_data: {gyro_data}")
  
  # delay for terminal output
  # time.sleep(1)
  
  # time between 2 consecutive update of filter
  dt = time.time() - start_time
  
  # complementary filter
  pitchFromAccel = math.atan2(-accel_data['x'] / math.sqrt(math.pow(accel_data['y'], 2) + math.pow(accel_data['z'], 2)))
  rollFromAccel = math.atan2(accel_data['y'] / math.sqrt(math.pow(accel_data['x'], 2) + math.pow(accel_data['z'], 2)))
  
  # make sure is float
  complementaryPitch = 0.0
  complementaryRoll = 0.0
  
  comp_filter_gain = 0.98
  
  complementaryPitch = comp_filter_gain * (complementaryPitch + (gyro_data['y'] * dt)) + (1 - comp_filter_gain) * pitchFromAccel
  complementaryRoll = comp_filter_gain * (complementaryRoll + (gyro_data['x'] * dt)) + (1 - comp_filter_gain) * rollFromAccel
  
  
  # orientation, calibrate?, clean data