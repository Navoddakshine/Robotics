from machine import SoftI2C, Pin
from time import sleep
import mpu6050

# Constants
sensitivity = 131  # Sensitivity of gyroscope
wheel_radius = 5  # Radius of the wheels in centimeters

# Initializations
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))
last_time = time.ticks_ms()
last_gyx = 0
last_gyy = 0
x_velocity = 0
y_velocity = 0

while True:
    # Read gyroscope values
    mpu = mpu6050.accel(i2c)
    gyx = mpu.get_values()["GyX"] / sensitivity
    gyy = mpu.get_values()["GyY"] / sensitivity

    # Calculate time interval
    current_time = time.ticks_ms()
    dt = (current_time - last_time) / 1000.0  # Convert to seconds

    # Estimate changes in orientation
    delta_gyx = gyx - last_gyx
    delta_gyy = gyy - last_gyy

    # Integrate changes to estimate velocities
    x_velocity += delta_gyx * wheel_radius
    y_velocity += delta_gyy * wheel_radius

    # Update last gyro values
    last_gyx = gyx
    last_gyy = gyy

    # Print velocities
    print("X Velocity:", x_velocity)
    print("Y Velocity:", y_velocity)

    # Sleep for some time
    sleep(0.1)  # Adjust sleep time as needed
