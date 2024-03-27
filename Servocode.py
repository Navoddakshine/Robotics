#####FOR TESTING SERVO 
import machine
import utime

SERVO_PIN = 14

servo = machine.PWM(machine.Pin(SERVO_PIN), freq=50)

def set_angle(angle):
    duty = int(utime.map_range(angle, 0, 180, 40, 115))
    servo.duty(duty)

set_angle(90)

#####CODE FOR THE FOUR SERVOS

import machine
import utime
#PINS FOR SERVO
SERVO_LEFT_FRONT_PIN = 4
SERVO_LEFT_BACK_PIN = 5
SERVO_RIGHT_FRONT_PIN = 6
SERVO_RIGHT_BACK_PIN = 7

SERVO_MIN_ANGLE = 0  
SERVO_MAX_ANGLE = 180 
SERVO_MIN_PULSE_WIDTH = 500  
SERVO_MAX_PULSE_WIDTH = 2500 

#Initializing
servo_front_left = machine.PWM(machine.Pin(SERVO_LEFT_FRONT_PIN), freq=50)
servo_front_right = machine.PWM(machine.Pin(SERVO_RIGHT_FRONT_PIN), freq=50)
servo_back_left = machine.PWM(machine.Pin(SERVO_LEFT_BACK_PIN = 5), freq=50)
servo_back_right = machine.PWM(machine.Pin(SERVO_RIGHT_BACK_PIN), freq=50)

def set_servo_angle(servo, angle):
    pulse_width = int(utime.map_range(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH))
    servo.duty(pulse_width)

def move_forward():
    set_servo_angle(servo_front_left, 90)
    set_servo_angle(servo_front_right, 90)
    set_servo_angle(servo_back_left, 90)
    set_servo_angle(servo_back_right, 90)

def move_backward():
    set_servo_angle(servo_front_left, 90)
    set_servo_angle(servo_front_right, 90)
    set_servo_angle(servo_back_left, 90)
    set_servo_angle(servo_back_right, 90)

def turn_left():
    set_servo_angle(servo_front_left, 0)
    set_servo_angle(servo_front_right, 180)
    set_servo_angle(servo_back_left, 180)
    set_servo_angle(servo_back_right, 0)

def turn_right():
    set_servo_angle(servo_front_left, 180)
    set_servo_angle(servo_front_right, 0)
    set_servo_angle(servo_back_left, 0)
    set_servo_angle(servo_back_right, 180)

move_forward()
utime.sleep(2) #forward movement for 2 seconds ish
turn_left()
utime.sleep(1) #and so on
