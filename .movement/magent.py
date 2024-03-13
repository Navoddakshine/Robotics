# main.py

from machine import Pin
import time

#CHANGE PINS
ELECTROMAGNET_PIN = 0
LED_PIN = 13

electromagnet = Pin(ELECTROMAGNET_PIN, Pin.OUT)
led = Pin(LED_PIN, Pin.OUT)

def setup():
    print("Electromagnet setup complete")
setup()

while True:
    # Turn on electromagnet and LED
    electromagnet.on()
    led.on()
    time.sleep(1)

    # Turn off electromagnet and LED
    electromagnet.off()
    led.off()
    time.sleep(1)
