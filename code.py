import RPi.GPIO as GPIO
import time
import smbus
import math

# Constants for MPU6050
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# GPIO Pins
VIBRATION_SENSOR_PIN = 17  # GPIO pin connected to the vibration sensor for tilt detection
ALCOHOL_SENSOR_PIN    = 16  # GPIO pin connected to the vibration sensor for alcohol detection
LED_PIN = 18     # GPIO pin connected to the LED

# Setup GPIO
GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
GPIO.setup(VIBRATION_SENSOR_PIN, GPIO.IN)  # Set the vibration sensor pin as an input
GPIO.setup(ALCOHOL_SENSOR_PIN, GPIO.IN)     # Set the alcohol sensor pin as an input
GPIO.setup(LED_PIN, GPIO.OUT)    # Set the LED pin as an output

# Initialize MPU6050
def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def compute_tilt_angles(ax, ay, az):
    pitch = math.atan2(ay, math.sqrt(ax*2 + az*2)) * 180 / math.pi
    roll = math.atan2(-ax, az) * 180 / math.pi
    return pitch, roll

# Initialize MPU6050
bus = smbus.SMBus(1)
Device_Address = 0x68

MPU_Init()

print("Starting sensor monitoring...")

try:
    while True:
        # Read Accelerometer raw values
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        
        # Convert raw values to g
        Ax = acc_x / 16384.0
        Ay = acc_y / 16384.0
        Az = acc_z / 16384.0
        
        # Compute tilt angles
        pitch, roll = compute_tilt_angles(Ax, Ay, Az)
        
        # Vibration and tilt detection
        if GPIO.input(VIBRATION_SENSOR_PIN):  # If the sensor detects vibration
            GPIO.output(LED_PIN, GPIO.HIGH)  # Turn on the LED
            print("Vibration detected!")
        else:
            GPIO.output(LED_PIN, GPIO.LOW)   # Turn off the LED
        
        # Display tilt status
        if abs(pitch) >= 45 or abs(roll) >= 45:
            print(f"Tilted! Pitch: {pitch:.2f} degrees, Roll: {roll:.2f} degrees")
        else:
            print(f"Pitch: {pitch:.2f} degrees, Roll: {roll:.2f} degrees")
        
        # Alcohol detection logic
        if GPIO.input(ALCOHOL_SENSOR_PIN):
            print("Alcohol not detected")
        else:
            print("Alcohol detected")
        
        time.sleep(1)  # Delay to avoid excessive CPU usage

except KeyboardInterrupt:
    print("Program terminated")

finally:
    GPIO.cleanup()  # Clean up GPIO settings