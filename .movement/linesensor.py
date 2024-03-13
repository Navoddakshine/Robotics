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

# LEDs to indicate position
right = Pin(14, Pin.OUT)
left = Pin(4, Pin.OUT)

while True:
    s1value = sensor1.read()
    s2value = sensor2.read()
    s3value = sensor3.read()
    s4value = sensor4.read()
    s5value = sensor5.read()

    # weighted average of all the sensors
    totalval = (0*s1value + 1000*s2value + 2000*s3value + 3000*s4value + 4000*s5value) / (s1value+s2value+s3value+s4value+s5value+0.01)

    #light LEDs to show position wrt the line
    right.value(0)
    left.value(0)

    if totalval>2500:
        right.value(1)

    elif totalval<1500:
        left.value(1)

    print("values:")
    print(s1value)
    print(s2value)
    print(s3value)
    print(s4value)
    print(s5value)
    print(totalval)

    sleep(1)  # 1 measurement per second
