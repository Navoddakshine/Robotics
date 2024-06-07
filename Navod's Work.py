
from machine import Pin, time_pulse_us, I2C
import time

magnet=Pin(15,Pin.OUT)

TOF10120_ADDRESS = 0x52  # I2C address of TOF10120 sensor

# Initialize I2C bus
i2c = I2C(scl=Pin(22), sda=Pin(21))

# TCS230 or TCS3200 pins wiring to ESP32
S0_PIN = 18
S1_PIN = 19
S2_PIN = 33
S3_PIN = 25
SENSOR_OUT_PIN = 32

# Stores frequency read by the photodiodes
red_frequency = 0
green_frequency = 0
blue_frequency = 0
black_frequency = 0

# Number of samples to average
NUM_SAMPLES = 10
red_samples = []
green_samples = []
blue_samples = []
black_samples = []

# Setting the outputs
s0_pin = Pin(S0_PIN, Pin.OUT)
s1_pin = Pin(S1_PIN, Pin.OUT)
s2_pin = Pin(S2_PIN, Pin.OUT)
s3_pin = Pin(S3_PIN, Pin.OUT)

s0_pin.value(1)
s1_pin.value(0)

# Setting the sensorOut as an input
sensor_out_pin = Pin(SENSOR_OUT_PIN, Pin.IN)

# Setting frequency scaling to 20%

def pulse_in(pin, level, timeout_us=100000):
    # Measure the duration until the pin changes its level
    return time_pulse_us(pin, level, timeout_us)

def average(samples):
    # Calculate the average of the samples
    return sum(samples) / len(samples)



color=''

while True:
    
    # Clear previous samples
    red_samples.clear()
    green_samples.clear()
    blue_samples.clear()
    black_samples.clear()


    # Collect samples for red frequency
    for _ in range(NUM_SAMPLES):
        # Setting RED (R) filtered photodiodes to be read
        s2_pin.value(0)
        s3_pin.value(0)
        
        # Reading the output frequency
        red_samples.append(pulse_in(sensor_out_pin, 1))

    # Calculate average red frequency
    red_frequency = average(red_samples)


    # Collect samples for green frequency
    for _ in range(NUM_SAMPLES):
        # Setting GREEN (G) filtered photodiodes to be read
        s2_pin.value(1)
        s3_pin.value(1)
        
        # Reading the output frequency
        green_samples.append(pulse_in(sensor_out_pin, 1))

    # Calculate average green frequency
    green_frequency = average(green_samples)


    # Collect samples for blue frequency
    for _ in range(NUM_SAMPLES):
        # Setting BLUE (B) filtered photodiodes to be read
        s2_pin.value(0)
        s3_pin.value(1)
        
        # Reading the output frequency
        blue_samples.append(pulse_in(sensor_out_pin, 1))
        
        
    # Calculate average blue frequency
    blue_frequency = average(blue_samples)
    
    # Collect samples for black frequency
    for _ in range(NUM_SAMPLES):
        # Setting BLUE (B) filtered photodiodes to be read
        s2_pin.value(1)
        s3_pin.value(0)
        
        # Reading the output frequency
        black_samples.append(pulse_in(sensor_out_pin, 1))
    
    # Calculate average black frequency
    black_frequency = average(black_samples)
    
    
#
    
    
    if red_frequency < 300 and blue_frequency > 300 and green_frequency > 300 and black_frequency < 200:
        color='Red'
        print("Red")
    elif green_frequency > 490 and blue_frequency > 460 and red_frequency > 520 and black_frequency < 200:
        color='Green'
        print("Green")
    elif green_frequency < 420 and blue_frequency < 420 and red_frequency > green_frequency and black_frequency < 200:
        color+'Green'
        print("GREEN CLOSE--------")
        
        
    elif green_frequency > 500 and blue_frequency < 350 and red_frequency > 380 and black_frequency < 200:
        color='Blue'
        print("Blue")
    elif red_frequency < 400 and blue_frequency < 380 and green_frequency > 480 and black_frequency < 200:
        color='Blue'
        print("BLUE CLOSEEE")
        
    elif blue_frequency > 600 and green_frequency > 600 and red_frequency >600 and black_frequency < 300:
        color='Black'
        print("Black")
    

        
    #print("Red:", red_frequency, "Green:", green_frequency, "Blue:", blue_frequency, "Black:", black_frequency)
    time.sleep_ms(200)

