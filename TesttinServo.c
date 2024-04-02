
--------------------------------------------------CODE WORK BUT INSTANTLY MOVES TO NEXT PWM-----------------------------
  #include <ESP32Servo.h>

#define SERVO_PIN_LEFT 13  // Pin 18 for left servo
#define SERVO_PIN_RIGHT 14  // Pin 19 for right servo

Servo servoLeft;
Servo servoRight;

void setup() {
  // Attach servos to pins
  servoLeft.attach(SERVO_PIN_LEFT);
  servoRight.attach(SERVO_PIN_RIGHT);
}

void loop() {
  // Define PWM values for normal, outward, and inward positions
  int pwm_normal = 1500;
  int pwm_outward = 1000;
  int pwm_inward = 2000;

  // Move both servos to normal state (centered)
  servoLeft.writeMicroseconds(pwm_normal);
  servoRight.writeMicroseconds(pwm_normal);
  delay(1000); // Adjust delay as needed

  // Move both servos outward
  servoLeft.writeMicroseconds(pwm_outward);
  servoRight.writeMicroseconds(pwm_outward);
  delay(1000); // Adjust delay as needed

  // Move both servos inward
  servoLeft.writeMicroseconds(pwm_inward);
  servoRight.writeMicroseconds(pwm_inward);
  delay(1000); // Adjust delay as needed
}


------------------------------------------------------CODE FOR MOVING SMOOTHLY (CODE #1)
#include <ESP32Servo.h>

#define SERVO_PIN_LEFT 13  // Pin 13 for left servo
#define SERVO_PIN_RIGHT 14  // Pin 14 for right servo

Servo servoLeft;
Servo servoRight;

void setup() {
  // Attach servos to pins
  servoLeft.attach(SERVO_PIN_LEFT);
  servoRight.attach(SERVO_PIN_RIGHT);
}

void moveServosSmoothly(int currentPwmLeft, int currentPwmRight, int targetPwmLeft, int targetPwmRight, int steps, int duration) {
  int incrementLeft = (targetPwmLeft - currentPwmLeft) / steps;
  int incrementRight = (targetPwmRight - currentPwmRight) / steps;
  for (int i = 0; i < steps; i++) {
    currentPwmLeft += incrementLeft;
    currentPwmRight += incrementRight;
    servoLeft.writeMicroseconds(currentPwmLeft);
    servoRight.writeMicroseconds(currentPwmRight);
    delay(duration / steps);
  }
}

void loop() {
  // Define PWM values for normal, outward, and inward positions
  int pwm_normal = 1500;
  int pwm_outward = 1000;
  int pwm_inward = 2000;

  // Move both servos to normal state (centered) smoothly
  moveServosSmoothly(pwm_outward, pwm_outward, pwm_normal, pwm_normal, 20, 1000);

  // Move both servos outward smoothly
  moveServosSmoothly(pwm_normal, pwm_normal, pwm_outward, pwm_outward, 20, 1000);

  // Move both servos inward smoothly
  moveServosSmoothly(pwm_outward, pwm_outward, pwm_inward, pwm_inward, 20, 1000);

  // Move both servos to normal state (centered) smoothly
  moveServosSmoothly(pwm_inward, pwm_inward, pwm_normal, pwm_normal, 20, 1000);
}

------------------------------------CODE FOR MOVING SMOOTHLY(#2)------------------------------
#include <ESP32Servo.h>

#define SERVO_PIN_LEFT 13  // Pin 13 for left servo
#define SERVO_PIN_RIGHT 14  // Pin 14 for right servo

Servo servoLeft;
Servo servoRight;

void setup() {
  // Attach servos to pins
  servoLeft.attach(SERVO_PIN_LEFT);
  servoRight.attach(SERVO_PIN_RIGHT);
}

void moveServosSmoothly(int currentPwmLeft, int currentPwmRight, int targetPwmLeft, int targetPwmRight, int steps, int duration) {
  int incrementLeft = (targetPwmLeft - currentPwmLeft) / steps;
  int incrementRight = (targetPwmRight - currentPwmRight) / steps;
  for (int i = 0; i < steps; i++) {
    currentPwmLeft += incrementLeft;
    currentPwmRight += incrementRight;
    servoLeft.writeMicroseconds(currentPwmLeft);
    servoRight.writeMicroseconds(currentPwmRight);
    delay(duration / steps);
  }
}

void loop() {
  // Define PWM values for normal, outward, and inward positions
  int pwm_normal = 1500;
  int pwm_outward = 1000;
  int pwm_inward = 2000;

  // Move both servos to normal state (centered) smoothly
  moveServosSmoothly(pwm_normal, pwm_normal, pwm_normal, pwm_normal, 20, 1000);

  // Move both servos outward smoothly
  moveServosSmoothly(pwm_normal, pwm_normal, pwm_outward, pwm_outward, 20, 1000);

  // Move both servos inward smoothly
  moveServosSmoothly(pwm_outward, pwm_outward, pwm_inward, pwm_inward, 20, 1000);
}
-----------------------------------------------------------------------------------------------------------------------------
