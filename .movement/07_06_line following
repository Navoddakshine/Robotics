from dcmotor import DCMotor
from machine import Pin, PWM, ADC
from time import sleep

timer = 0

#speeds
straightspeed = 40
ninetyforward = 25
ninetyturn = 40

#set up light sensor

sensor1 = ADC(Pin(15))
sensor1.atten(ADC.ATTN_11DB)
sensor2 = ADC(Pin(2))
sensor2.atten(ADC.ATTN_11DB)
sensor3 = ADC(Pin(4))
sensor3.atten(ADC.ATTN_11DB)
sensor4 = ADC(Pin(34))
sensor4.atten(ADC.ATTN_11DB)
sensor5 = ADC(Pin(35))
sensor5.atten(ADC.ATTN_11DB)

""" s1low = 3525
s1high = 4095
s2low = 2440
s2high = 3950
s3low = 3080
s3high = 4095
s4low = 3900
s4high = 4095
s5low = 3927
s5high = 4095

scalelow = 1000
scalehigh = 5000 """

white = 4040

""" leftWhite = 4000
centerWhite = 3950
rightWhite = 3900 """

state = "straight"

# setup pid
delta_t = 0.2
desired_line_value = 1999.996
kp = 2
ki = 0
kd = 0.3

prev_error = 0
accu_error = 0

# setup motors

frequency = 15000

pin1 = Pin(12, Pin.OUT)
pin2 = Pin(13, Pin.OUT)
enable = PWM(Pin(32), frequency)

pin3 = Pin(14, Pin.OUT)
pin4 = Pin(27, Pin.OUT)
enable2 = PWM(Pin(18), frequency)

motor_left = DCMotor(pin1, pin2, enable, 500, 1023)
motor_right = DCMotor(pin3, pin4, enable2, 500, 1023)
#Set min duty cycle (350) and max duty cycle (1023)
#dc_motor = DCMotor(pin1, pin2, enable, 350, 1023)

# FUNCTIONS

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

def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

sleep(1)

while True:
    s1value = sensor1.read()
    #s1scaled = scale_value(s1value, s1low, s1high, scalelow, scalehigh)

    s2value = sensor2.read()+1300
    #s2scaled = scale_value(s2value, s2low, s2high, scalelow, scalehigh)
    if s2value > 4095:
        s2value = 4095

        #  sensor2 is broken and consistently outputs a value ~ 1000 lower than the other sensors ; for sake of average, manually adjusted

    s3value = sensor3.read()
    #s3scaled = scale_value(s3value, s3low, s3high, scalelow, scalehigh)
    
    s4value = sensor4.read()
    #s4scaled = scale_value(s4value, s4low, s4high, scalelow, scalehigh)
    
    s5value = sensor5.read()
    #s5scaled = scale_value(s5value, s5low, s5high, scalelow, scalehigh)

    if state =="straight":
        if s1value < white:
            timer = 100
            state = "left"
        if s5value < white:
            timer = 100
            state = "right"
    elif state =="left" and timer <= 0:
        timer = 80
        state = "leftturn"

    elif state =="right" and timer <= 0:
        timer = 80
        state = "rightturn"

    if state == "straight":

        #total_line_value = (0*s1scaled + 1000*s2scaled + 2000*s3scaled + 3000*s4scaled + 4000*s5scaled) / (s1scaled+s2scaled+s3scaled+s4scaled+s5scaled+0.01)
        total_line_value = (0*s1value + 1000*s2value + 2000*s3value + 3000*s4value + 4000*s5value) / (s1value+s2value+s3value+s4value+s5value+0.01)

        line_error = get_line_error(desired_line_value, total_line_value)

        pid_output, prev_error, accu_error = pid_controller(line_error,prev_error,accu_error, kp, kd, ki)
        #scaled_pid = scale_value(pid_output, -470+accu_error, 800+accu_error, -400, 400)
        
        print(pid_output)
        if pid_output < -20+accu_error:
            pid_output = -20
        elif pid_output > 60+accu_error:
            pid_output = 60

        #print(s1value, s2value, s3value, s4value, s5value)
        #print(s1scaled, s2scaled, s3scaled, s4scaled, s5scaled)
        #print(total_line_value, line_error, pid_output)


        speed = scale_value(pid_output, -20+accu_error, 20+accu_error, -straightspeed, straightspeed)
        motor_left.forward(straightspeed+speed)
        motor_right.forward(straightspeed-speed)

        print(total_line_value)
        print(pid_output)

    elif state == "left":
        print("left")
        motor_left.forward(ninetyforward)
        motor_right.forward(ninetyforward)
        timer = timer-1

    elif state == "leftturn":
        print("lefturn")
        motor_left.backwards(ninetyturn)
        motor_right.forward(ninetyturn)
        timer = timer-1
        if timer <= 0:
            if s3value < white:
                state ="straight"

    elif state == "right":
        print("right")
        motor_left.forward(ninetyforward)
        motor_right.forward(ninetyforward)
        timer = timer-1

    elif state == "rightturn":
        print("rightturn")
        motor_left.forward(ninetyturn)
        motor_right.backwards(ninetyturn)
        timer = timer-1
        if timer <= 0:
            if s3value < white:
                state ="straight"
    
    print(s1value,s2value,s3value,s4value,s5value)
    sleep(0.01)
