import machine
import LineSensor
import PID

#Initialize all hardware components
PID.init()
LineSensor.init()

class StateMachine:
    def __init__(self):
        self.current_state = "forward"
    def update_state(self, newstate):
        self.current_state = newstate
    def execute_state(self):
        if self.current_state == "forward":
            self.forward()
        elif self.current_state == "backward":
            self.backward()
        elif self.current_state == "turn_left":
            self.turn_left()
        elif self.current_state == "turn_right":
            self.turn_right()
    def forward(self):
        pass
    def backward(self):
        pass
    def turn_left(self):
        pass
    def turn_right(self):
        pass

def main():
    while True:


if __name__ == "__main__":
    main()
