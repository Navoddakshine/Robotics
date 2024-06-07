import machine
import time

# Define the pin where the IR sensor is connected
ir_sensor = machine.Pin(27, machine.Pin.IN)

while True:
    # Read the sensor value
    sensor_value = ir_sensor.value()
    
    # Print the sensor value
    print("IR Sensor Value:", sensor_value)
    
    # Wait for 0.1 seconds before reading again
    time.sleep(0.1)

If sensor value 0 then detected else not. 
