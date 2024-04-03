# main.py -- put your code here!

from machine import Pin, ADC, UART
from time import sleep

sensor1 = ADC(Pin(25))
sensor1.atten(ADC.ATTN_11DB)
sensor2 = ADC(Pin(26))
sensor2.atten(ADC.ATTN_11DB)
sensor3 = ADC(Pin(27))
sensor3.atten(ADC.ATTN_11DB)
sensor4 = ADC(Pin(34))
sensor4.atten(ADC.ATTN_11DB)
sensor5 = ADC(Pin(35))
sensor5.atten(ADC.ATTN_11DB)

delta_t = 0.2
desired_line_value = 1999.998
kp = 5
ki = 0.01
kd = 2.5

prev_error = 0
accu_error = 0

# line error
def get_line_error(desired_line_value, total_line_value):
    line_error = desired_line_value - total_line_value
    return line_error

# pid controller
def pid_controller(error, prev_error, accu_error, kp, kd, ki):
    P = kp * error                      # Proportional term; kp is the proportional gain
    I = accu_error + ki * error     # Intergral term; ki is the integral gain
    D = kd * (error - prev_error)   # Derivative term; kd is the derivative gain
    
    output = P + I + D              # controller output
    
    # store values for the next iteration
    prev_error = error     # error value in the previous interation (to calculate the derivative term)
    accu_error = I      # accumulated error value (to calculate the integral term)
    
    return output, prev_error, accu_error

# remap values
def scale_value(unscaled, from_min, from_max, to_min, to_max):
    return (to_max-to_min)*(unscaled-from_min)/(from_max-from_min)+to_min

while True:
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
        print("go backwards lol")

    total_line_value = (0*s1value + 1000*s2value + 2000*s3value + 3000*s4value + 4000*s5value) / (s1value+s2value+s3value+s4value+s5value+0.01)

    line_error = get_line_error(desired_line_value, total_line_value)
    #output, prev_error, accu_error = pid_controller(line_error, prev_error, accu_error, kp, kd, ki)

    desired_angle = scale_value(line_error, -325, 325, -45, 45)

    #print(s1value, s2value, s3value, s4value, s5value)
    print(total_line_value, line_error, desired_angle)

    sleep(0.2) 
