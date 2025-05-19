import time
import sys
from pad4pi import rpi_gpio
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

# Setup I2C for PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Define motor channels on PCA9685 (0-3)
motors = [0, 1, 2, 3]

# Speed mapping in microseconds pulse width for ESC throttle signals
speed_levels = {
    "0": 0,
    "1": 1000,
    "2": 1125,
    "3": 1250,
    "4": 1375,
    "5": 1500,
    "6": 1625,
    "7": 1750,
    "8": 1875,
    "9": 2000,
}

# Keep track of each motor's current pulse width (start at idle 1000)
pulsewidths = [1000, 1000, 1000, 1000]

selected_motor = None

def set_motor(channel, pulse_us):
    if pulse_us == 0:
        pca.channels[channel].duty_cycle = 0
    else:
        # Convert pulse width in microseconds to 12-bit duty cycle value
        pulse_length = int(pulse_us * 4096 / 20000)  # 20ms period at 50Hz
        pca.channels[channel].duty_cycle = pulse_length

def stop_all():
    global pulsewidths
    for i, ch in enumerate(motors):
        pulsewidths[i] = 0
        set_motor(ch, 0)
    print("All motors stopped.")

# Keypad setup
KEYPAD = [
    ["1","2","3","A"],
    ["4","5","6","B"],
    ["7","8","9","C"],
    ["*","0","#","D"]
]

# BCM GPIO pins for keypad rows and columns (adjust if needed)
ROW_PINS = [12,16,20,21]
COL_PINS = [24,25,8,7]

factory = rpi_gpio.KeypadFactory()
keypad = factory.create_keypad(keypad=KEYPAD, row_pins=ROW_PINS, col_pins=COL_PINS)

def process_key(key):
    global selected_motor

    print(f"Key pressed: {key}")

    if key in ["A", "B", "C", "D"]:
        selected_motor = {"A":0, "B":1, "C":2, "D":3}[key]
        print(f"Motor {selected_motor+1} selected.")

    elif key in speed_levels.keys():
        if selected_motor is not None:
            pulsewidths[selected_motor] = speed_levels[key]
            set_motor(motors[selected_motor], speed_levels[key])
            print(f"Motor {selected_motor+1} speed set to {speed_levels[key]} µs.")
        else:
            print("Select a motor first (A-D).")

    elif key == "#":
        selected_motor = None
        print("Motor deselected.")

    elif key == "*":
        stop_all()
        print("All motors stopped. Exiting program.")
        keypad.cleanup()
        pca.deinit()
        sys.exit(0)

    else:
        print("Invalid key.")

try:
    print("Ready! Press A-D to select motor, 0-9 to set speed, * to stop all and exit, # to deselect motor.")
    keypad.registerKeyPressHandler(process_key)

    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    stop_all()
    keypad.cleanup()
    pca.deinit()
