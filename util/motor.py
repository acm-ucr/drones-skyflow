import pigpio
import time

# gpio pin connected to the esc signal wire

#ESC_PIN = 15

MOTOR_1=15
MOTOR_2=7
MOTOR_3=15
MOTOR_4=20

# FRONT
#  1   2 
#   \ /
#    |
#   / \
#  3   4
# BACK


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

# # gradually increase throttle
# for pulsewidth in range(1000, 1400, 100):
#     pi.set_servo_pulsewidth(ESC_PIN, pulsewidth)
#     print(f"Pulsewidth: {pulsewidth}µs")
#     time.sleep(1)

# # gradually decrease throttle
# for pulsewidth in range(1400, 1000, -100):
#     pi.set_servo_pulsewidth(ESC_PIN, pulsewidth)
#     print(f"Pulsewidth: {pulsewidth}µs")
#     time.sleep(1)

user_choice = int(input('Individual Control Mode: 1...Group Control Mode: 2 :: '))

if(user_choice==1):
    user_input = int(input('MOTOR 1: 1(+), 2(-)...MOTOR 2: 3(+), 4(-)...MOTOR 3: 5(+), 6(-)...MOTOR 4: 7(+), 8(-): '))
    pulsewidth_1=1000
    pulsewidth_2=1000
    pulsewidth_3=1000
    pulsewidth_4=1000
    while user_input!=0:
        if(user_input==1):
            pulsewidth_1+=25
            pi.set_servo_pulsewidth(MOTOR_1,pulsewidth_1)
        if(user_input==2):
            pulsewidth_1-=25
            pi.set_servo_pulsewidth(MOTOR_1,pulsewidth_1)
        if(user_input==3):
            pulsewidth_2+=25
            pi.set_servo_pulsewidth(MOTOR_2,pulsewidth_2)
        if(user_input==4):
            pulsewidth_2-=25
            pi.set_servo_pulsewidth(MOTOR_2,pulsewidth_2)
        if(user_input==5):
            pulsewidth_3+=25
            pi.set_servo_pulsewidth(MOTOR_3,pulsewidth_2)
        if(user_input==6):
            pulsewidth_3-=25
            pi.set_servo_pulsewidth(MOTOR_3,pulsewidth_2)
        if(user_input==7):
            pulsewidth_4+=25
            pi.set_servo_pulsewidth(MOTOR_4,pulsewidth_2)
        if(user_input==8):
            pulsewidth_4-=25
            pi.set_servo_pulsewidth(MOTOR_4,pulsewidth_2)
        print(f"Motor 1 Pulsewidth: {pulsewidth_1}µs")
        print(f"Motor 2 Pulsewidth: {pulsewidth_2}µs")
        print(f"Motor 3 Pulsewidth: {pulsewidth_3}µs")
        print(f"Motor 4 Pulsewidth: {pulsewidth_4}µs")
        user_input = int(input('MOTOR 1: 1(+), 2(-)...MOTOR 2: 3(+), 4(-)...MOTOR 3: 5(+), 6(-)...MOTOR 4: 7(+), 8(-): '))


    print("Motor test stopped by (0)")

elif(user_choice==2):
    user_input = int(input('MOTORS: 1(+) 2(-): '))
    pulsewidth=1000
    while user_input!=0:
        if(user_input==1):
            pulsewidth+=25
            pi.set_servo_pulsewidth(MOTOR_1,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_2,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_3,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_4,pulsewidth)
        if(user_input==2):
            pulsewidth-=25
            pi.set_servo_pulsewidth(MOTOR_1,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_2,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_3,pulsewidth)
            pi.set_servo_pulsewidth(MOTOR_4,pulsewidth)
        print(f"Motor 1 Pulsewidth: {pulsewidth}µs")
        print(f"Motor 2 Pulsewidth: {pulsewidth}µs")
        print(f"Motor 3 Pulsewidth: {pulsewidth}µs")
        print(f"Motor 4 Pulsewidth: {pulsewidth}µs")
        user_input = int(input('MOTORS: 1(+) 2(-): '))

elif user_choice == 3: #
    # Initialize all motors to base idle throttle
    base_throttle = 1000
    spin_throttle_delta = 50  # You can tweak this value for more or less rotation
    spin_time = 1           # Time to rotate in seconds (adjust as needed)

    # Apply rotation
    pi.set_servo_pulsewidth(MOTOR_1, base_throttle + spin_throttle_delta)  # CW
    pi.set_servo_pulsewidth(MOTOR_4, base_throttle + spin_throttle_delta)  # CW

    pi.set_servo_pulsewidth(MOTOR_2, base_throttle - spin_throttle_delta)  # CCW
    pi.set_servo_pulsewidth(MOTOR_3, base_throttle - spin_throttle_delta)  # CCW

    print("Rotating clockwise...")
    time.sleep(spin_time)

    # Reset all to base throttle
    pi.set_servo_pulsewidth(MOTOR_1, base_throttle)
    pi.set_servo_pulsewidth(MOTOR_2, base_throttle)
    pi.set_servo_pulsewidth(MOTOR_3, base_throttle)
    pi.set_servo_pulsewidth(MOTOR_4, base_throttle)

    print("Rotation complete. Motors reset to idle.")



# stop motor
pi.set_servo_pulsewidth(MOTOR_1, 1000)
time.sleep(1)

pi.set_servo_pulsewidth(MOTOR_2, 1000)
time.sleep(1)

pi.set_servo_pulsewidth(MOTOR_3, 1000)
time.sleep(1)

pi.set_servo_pulsewidth(MOTOR_4, 1000)
time.sleep(1)

# shut down pigpio
pi.set_servo_pulsewidth(MOTOR_1, 0)
pi.stop()

pi.set_servo_pulsewidth(MOTOR_2, 0)
pi.stop()

pi.set_servo_pulsewidth(MOTOR_3, 0)
pi.stop()

pi.set_servo_pulsewidth(MOTOR_4, 0)
pi.stop()

print("Motor test complete.")
