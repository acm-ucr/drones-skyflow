import RPi.GPIO as GPIO
import time


#CHANGE PIN NUMBER
ESC_PIN = 18  

GPIO.setmode(GPIO.BCM)
GPIO.setup(ESC_PIN, GPIO.OUT)

pwm = GPIO.PWM(ESC_PIN, 50)  
pwm.start(0) 

def set_speed(speed):
    #Speed should be a value between 0 and 100 
    duty_cycle = 2.5 + (speed / 100) * 10  
    pwm.ChangeDutyCycle(duty_cycle)
    print(f"Speed set to {speed}%")

try:
    set_speed(0)  # Ensure motor is stopped initially
    time.sleep(2)

    print("Starting motor...")
    set_speed(20)  # 20% speed
    time.sleep(3)

    set_speed(50)  # 50% speed
    time.sleep(3)

    set_speed(100)  # Full speed
    time.sleep(3)

    set_speed(0)  # Stop motor
    time.sleep(2)

finally:
    pwm.stop()
    GPIO.cleanup()
