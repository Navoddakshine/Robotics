from machine import Pin
import time

#CHANGE PIN
SENSOR_PIN = 7

line_sensor = Pin(SENSOR_PIN, Pin.IN)

def setup():
    print("Line sensor setup complete")

def loop():
    val = line_sensor.value()
    if val == 1:
        print("Line detected")
    else:
        print("Line NOT detected")
    time.sleep_ms(500)

setup()

while True:
    loop()
