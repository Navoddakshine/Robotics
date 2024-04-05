# main.py -- put your code here!

from machine import Pin
import time

#CHANGE PINS
MP1 = 18
M1 = Pin(MP1, Pin.OUT)

while True:
    # Turn on electromagnet and LED
    M1.on()
    print("on")
    time.sleep(1)

    # Turn off electromagnet and LED
    M1.off()
    print("off")
    time.sleep(1)
