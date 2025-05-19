import time
import board
import busio
import signal
import sys
from adafruit_pca9685 import PCA9685

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

try:
    # Define motor pins (channels) and their initial speeds and max speeds
    motor_channels = [0, 1, 2, 3]  # Assigning motors to channels 0, 5, 9, 13
    motor_speeds = {
        0: {"current": 1000, "max": 1500},  # Motor 1: Starts at 1000µs, max at 1400µs
        1: {"current": 1000, "max": 1500},  # Motor 2: Starts at 1000µs, max at 1500µs
        2: {"current": 1000, "max": 1600},  # Motor 3: Starts at 1000µs, max at 1600µs
        3: {"current": 1000, "max": 1300}  # Motor 4: Starts at 1000µs, max at 1700µs
    }

    # Arm all motors at their respective starting speeds
    print("Arming all ESCs at their starting speeds...")
    for motor_channel in motor_channels:
        pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(motor_speeds[motor_channel]["current"])
    time.sleep(5)


    pca.channels[0].duty_cycle = pulse_us_to_duty(1500)
    pca.channels[1].duty_cycle = pulse_us_to_duty(1500)
    pca.channels[2].duty_cycle = pulse_us_to_duty(1600)
    pca.channels[3].duty_cycle = pulse_us_to_duty(1300)

    time.sleep(6)

    pca.channels[0].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[1].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[2].duty_cycle = pulse_us_to_duty(1100)
    pca.channels[3].duty_cycle = pulse_us_to_duty(1100)

    time.sleep(0.5)

    pca.channels[0].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[1].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[2].duty_cycle = pulse_us_to_duty(1000)
    pca.channels[3].duty_cycle = pulse_us_to_duty(1000)


    # # Increase throttle on all motors simultaneously but at different speeds
    print("Increasing throttle on all motors...")
    # for us in range(1000, 2100, 50):  # General ramping for the motors
    #     print(f"Throttle: {us}µs")
    #     for motor_channel in motor_channels:
    #         # Increase each motor's speed separately
    #         if motor_speeds[motor_channel]["current"] < motor_speeds[motor_channel]["max"]:
    #             motor_speeds[motor_channel]["current"] = min(motor_speeds[motor_channel]["current"] + 50, motor_speeds[motor_channel]["max"])
    #         pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(motor_speeds[motor_channel]["current"])
    #     time.sleep(0.5)

    # Decrease throttle on all motors at different rates
    print("Decreasing throttle on all motors...")
    # for us in range(1500, 999, -50):
    #     print(f"Throttle: {us}µs")
    #     for motor_channel in motor_channels:
    #         # Decrease each motor's speed separately
    #         if motor_speeds[motor_channel]["current"] > 1000:
    #             motor_speeds[motor_channel]["current"] = max(motor_speeds[motor_channel]["current"] - 50, 1000)
    #         pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(motor_speeds[motor_channel]["current"])
    #     time.sleep(0.5)

    print("Stopping all motors.")
    # for motor_channel in motor_channels:
    #     pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(1000)
    # time.sleep(2)

    print("\nAll motors tested successfully.")

finally:
    # Final cleanup if not already stopped
    for motor_channel in motor_channels:
        pca.channels[motor_channel].duty_cycle = pulse_us_to_duty(1000)
    pca.deinit()
    print("PCA9685 deinitialized.")
