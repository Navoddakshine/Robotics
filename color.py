from machine import Pin
from machine import time_pulse_us
import time

class Color:
    def __init__(self, s0_pin, s1_pin, s2_pin, s3_pin, sensor_out_pin, num_samples=10):
        self.s0_pin = Pin(s0_pin, Pin.OUT)
        self.s1_pin = Pin(s1_pin, Pin.OUT)
        self.s2_pin = Pin(s2_pin, Pin.OUT)
        self.s3_pin = Pin(s3_pin, Pin.OUT)
        self.call_red = 700
        self.call_blue = 400
        self.call_green = 500
        self.call_black = 300
        self.sensor_out_pin = Pin(sensor_out_pin, Pin.IN)
        self.num_samples = num_samples
        self.red_samples = []
        self.green_samples = []
        self.blue_samples = []
        self.black_samples = []

        self.s0_pin.value(1)
        self.s1_pin.value(0)

    def pulse_in(self, pin, level, timeout_us=100000):
        pulse_width = time_pulse_us(pin, level, timeout_us)
        if pulse_width < 0:
            return 0
        return pulse_width

    def average(self, samples):
        return sum(samples) / len(samples) if samples else 0

    def read_color_frequencies(self):
        self.red_samples.clear()
        self.green_samples.clear()
        self.blue_samples.clear()
        self.black_samples.clear()

        for _ in range(self.num_samples):
            self.s2_pin.value(0)
            self.s3_pin.value(0)
            self.red_samples.append(self.pulse_in(self.sensor_out_pin, 1))

        for _ in range(self.num_samples):
            self.s2_pin.value(1)
            self.s3_pin.value(1)
            self.green_samples.append(self.pulse_in(self.sensor_out_pin, 1))

        for _ in range(self.num_samples):
            self.s2_pin.value(0)
            self.s3_pin.value(1)
            self.blue_samples.append(self.pulse_in(self.sensor_out_pin, 1))

        for _ in range(self.num_samples):
            self.s2_pin.value(1)
            self.s3_pin.value(0)
            self.black_samples.append(self.pulse_in(self.sensor_out_pin, 1))

        red_frequency = self.average(self.red_samples)
        green_frequency = self.average(self.green_samples)
        blue_frequency = self.average(self.blue_samples)
        black_frequency = self.average(self.black_samples)

        return red_frequency, green_frequency, blue_frequency, black_frequency

    def identifying_color(self, red_frequency, green_frequency, blue_frequency, black_frequency):
        color_ranges = {
            'Red': {
                'red_min': 100, 'red_max': 300,
                'green_min': 300, 'green_max': 500,
                'blue_min': 300, 'blue_max': 500,
                'black_max': 200
            },
            'Green': {
                'red_min': 520, 'red_max': 700,
                'green_min': 490, 'green_max': 700,
                'blue_min': 460, 'blue_max': 680,
                'black_max': 200
            },
            'Blue': {
                'red_min': 260, 'red_max': 300,
                'green_min': 370, 'green_max': 450,
                'blue_min': 250, 'blue_max': 390,
                'black_max': 200
            },
            'Black': {
                'red_min': 600, 'red_max': 800,
                'green_min': 600, 'green_max': 800,
                'blue_min': 600, 'blue_max': 800,
                'black_max': 300
            }
        }

        for color, ranges in color_ranges.items():
            if ('red_min' not in ranges or red_frequency >= ranges['red_min']) and \
               ('red_max' not in ranges or red_frequency <= ranges['red_max']) and \
               ('green_min' not in ranges or green_frequency >= ranges['green_min']) and \
               ('green_max' not in ranges or green_frequency <= ranges['green_max']) and \
               ('blue_min' not in ranges or blue_frequency >= ranges['blue_min']) and \
               ('blue_max' not in ranges or blue_frequency <= ranges['blue_max']) and \
               ('black_max' not in ranges or black_frequency <= ranges['black_max']):
                return color

        return 'Unknown'
