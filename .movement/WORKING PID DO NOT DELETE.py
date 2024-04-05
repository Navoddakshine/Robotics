# Complete project details at https://RandomNerdTutorials.com/micropython-esp32-esp8266-dc-motor-l298n/

from dcmotor import DCMotor
from machine import Pin, PWM, ADC
from time import sleep
from hcsr04.py import HCSR04

#set up light sensor

sensor1 = ADC(Pin(25))
sensor1.atten(ADC.ATTN_11DB)
sensor2 = ADC(Pin(33))
sensor2.atten(ADC.ATTN_11DB)
sensor3 = ADC(Pin(32))
sensor3.atten(ADC.ATTN_11DB)
sensor4 = ADC(Pin(35))
sensor4.atten(ADC.ATTN_11DB)
sensor5 = ADC(Pin(34))
sensor5.atten(ADC.ATTN_11DB)

#setup ultrasonic
us1 = HCSR04(trigger_pin=16, echo_pin=17, echo_timeout_us=10000)

#setup magnet
M1 = Pin(13, Pin.OUT)

# setup pid
delta_t = 0.2
desired_line_value = 1999.998
kp = 2
ki = 0
kd = 0.1

prev_error = 0
accu_error = 0

# setup motors

frequency = 15000

pin1 = Pin(3, Pin.OUT)
pin2 = Pin(2, Pin.OUT)
enable = PWM(Pin(14), frequency)

pin3 = Pin(26, Pin.OUT)
pin4 = Pin(27, Pin.OUT)
enable2 = PWM(Pin(12), frequency)

motor_left = DCMotor(pin1, pin2, enable)
motor_right = DCMotor(pin3, pin4, enable2)
#Set min duty cycle (350) and max duty cycle (1023)
#dc_motor = DCMotor(pin1, pin2, enable, 350, 1023)

# setup states
states = "forwards"

#functions

# line error
def get_line_error(desired_line_value, total_line_value):
    line_error = desired_line_value - total_line_value
    return line_error

# pid controller currently does NOT use delta_t - if weird errors, try adding that in first
def pid_controller(error, prev_error, accu_error, kp, kd, ki):
    P = kp * error                  # Proportional term; kp is the proportional gain
    I = accu_error + ki * error     # Intergral term; ki is the integral gain
    D = kd * (error - prev_error)   # Derivative term; kd is the derivative gain
    
    output = P + I + D              # controller output
    
    # store values for the next iteration
    prev_error = error  # error value in the previous interation (to calculate the derivative term)
    accu_error = I      # accumulated error value (to calculate the integral term)
    
    return output, prev_error, accu_error

# remap values
def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

while True:
usvalue = us1.distance_cm() # check distance always

    #transitions
    match states:
        case "forwards":
            if usvalue <= 1:
                states = "obstacle"

        case "obstacle":
            if usvalue => 1:
                states = "forwards"

    #effects
    match states: 
        case "forwards":
            
            s1value = sensor1.read()
            s2value = sensor2.read()
            s3value = sensor3.read()
            s4value = sensor4.read()
            s5value = sensor5.read()
        
            if s1value > 3000:
                s1value = 3000
            if s2value > 3000:
                s2value = 3000
            if s3value > 3000:
                s3value = 3000
            if s4value > 3000:
                s4value = 3000
            if s5value > 3000:
                s5value = 3000
            if s1value == 3000 and s2value == 3000 and s3value == 3000 and s4value == 3000 and s5value == 3000:
                motor_right.backwards(10)
                motor_left.backwards(10)
        
            total_line_value = (0*s1value + 1000*s2value + 2000*s3value + 3000*s4value + 4000*s5value) / (s1value+s2value+s3value+s4value+s5value+0.01)
        
            line_error = get_line_error(desired_line_value, total_line_value)
            output, prev_error, accu_error = pid_controller(line_error, prev_error, accu_error, kp, kd, ki)
            if output > 200:
                output = 200
            if output < -200:
                output = -200
        
            speed_variable = scale_value(output, -200+accu_error, 200+accu_error, -10, 10)
        
            """ line_error = get_line_error(desired_line_value, total_line_value)
            #output, prev_error, accu_error = pid_controller(line_error, prev_error, accu_error, kp, kd, ki)
        
            desired_angle = scale_value(line_error, -325, 325, -45, 45) """
        
            #print(s1value, s2value, s3value, s4value, s5value)
            print(total_line_value, line_error, output, accu_error, speed_variable)
            
            motor_right.forward(5-speed_variable)
            motor_left.forward(5+speed_variable)
            sleep(0.2)

        case "obstacle":
            motor_right.stop()
            motor_left.stop()
            M1.on()
            sleep(3)
            M1.off()

    print(states)
            

# motor_right.forward(50)
# motor_left.forward(50)
# sleep(2)
# motor_right.stop()
# motor_left.stop()
# sleep(3)
# motor_right.backwards(100)
# motor_left.backwards(100)
# sleep(2)
# motor_right.forward(5)
# motor_left.forward(5)
# sleep(5)
# motor_right.stop()
# motor_left.stop()

# from machine import Pin, PWM
# import time
# 
# # Motor pins
# motorRightPin1 = Pin(2, Pin.OUT)
# motorRightPin2 = Pin(3, Pin.OUT)
# #enable1RightPin = PWM(Pin(14))
# 
# motorLeftPin1 = Pin(26, Pin.OUT)
# motorLeftPin2 = Pin(27, Pin.OUT)
# #enable1LeftPin = PWM(Pin(12))
# 
# # PWM properties
# freq = 30000
# pwmChannel = 0
# resolution = 8
# dutyCycle = 200
# 
# # Setup PWM
# # enable1LeftPin.freq(freq)
# # enable1RightPin.freq(freq)
# 
# print("Testing DC Motor...")
# 
# def move_forward():
#     motorRightPin1.value(0)
#     motorRightPin2.value(1)
#     motorLeftPin1.value(0)
#     motorLeftPin2.value(1)
# #     enable1RightPin.duty(dutyCycle)
# #     enable1LeftPin.duty(dutyCycle)
#     print("help")
# 
# def stop_motors():
#     motorRightPin1.value(0)
#     motorRightPin2.value(0)
#     motorLeftPin1.value(0)
#     motorLeftPin2.value(0)
# #     enable1RightPin.duty(0)
# #     enable1LeftPin.duty(0)
# 
# while True:
#     # Move both motors forward at maximum speed
#     print("Moving Forward")
#     move_forward()
#     time.sleep(5)
# 
#     # Move DC motor forward with increasing speed
# #     motorRightPin1.value(1)
# #     motorRightPin2.value(0)
# #     motorLeftPin2.value(1)
# #     motorLeftPin1.value(0)
# #     while dutyCycle <= 255:
# #         enable1RightPin.duty(dutyCycle)
# #         enable1LeftPin.duty(dutyCycle)
# #         print("Forward with duty cycle:", dutyCycle)
# #         dutyCycle += 5
# #         time.sleep(0.5)
# # 
# #     dutyCycle = 200
#     stop_motors()
#     time.sleep(5)

