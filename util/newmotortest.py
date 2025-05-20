import time
import threading
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

def print_overview():
    try:
        while True:
            print("\n" + "="*30)
            print("         SkyFlow Overview")
            print("="*30)
            print(f"          Front")
            print()
            print(f"M3: {m1}µs     M0: {m2}µs")
            print("         \\   /")
            print("          \\ /")
            print("           |")
            print("          / \\")
            print("         /   \\")
            print(f"M2: {m3}µs     M1: {m4}µs")
            print("          Back")
            print("="*30 + "\n")
            time.sleep(5)
    except KeyboardInterrupt:
        print("Stopped by user.")

#Insert PID function here!!!!!

try:
    # Define motor pins (channels) and their initial speeds and max speeds
    motor_channels = [0, 1, 2, 3]  # Assigning motors to channels
    thread = threading.Thread(target=print_overview)
    thread.daemon = True  # dies when main program exits
    thread.start()
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
