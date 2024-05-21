/*



https://techtutorialsx.com/2018/03/09/esp32-arduino-getting-the-bluetooth-device-address/
https://github.com/pablomarquez76/PS4_Controller_Host/tree/main/examples
https://github.com/kimballa/button-debounce/blob/main/examples/buttons/buttons.ino


https://github.com/jkb-git/ESP32Servo/blob/master/examples/Multiple-Servo-Example-Arduino/Multiple-Servo-Example-Arduino.ino

MAC-Address: 30:AE:A4:99:FF:36

*/



/*###########################################
##  _    _ _                 _            ##
## | |  (_) |__ _ _ __ _ _ _(_)___ ___   ##
## | |__| | '_ \ '_/ _` | '_| / -_|_-<  ##
## |____|_|_.__/_| \__,_|_| |_\___/__/ ##
###################################################### Import libraries */
//#include <Arduino.h>
#include <WiFi.h>
//#include "esp_bt_main.h"
//#include "esp_bt_device.h"
#include <PS4Controller.h>
#include<debounce.h>
//#include <analogWrite.h>
#include <ESP32Servo.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/*##################################
##  ___      _                   ##
## |   \ ___| |__ _  _ __ _     ##
## | |) / -_) '_ \ || / _` |   ##
## |___/\___|_.__/\_,_\__, |  ##
##                    |___/  ##
############################################################ DEBUG MODES */
#define DEBUG_Servo 0
#define DEBUG_ARM 0

#define DEBUG_Trim 0
#define REPORT_ESC 0
#define REPORT_Offset 0

#define REPORT_Sticks 0
#define REPORT_triggers 0

int period = 1000;
unsigned long time_now = 0;

/*##############################
##  ___ _        ___ ___     ##
## | _ (_)_ _   |_ _/ _ \   ##
## |  _/ | ' \   | | (_) | ##
## |_| |_|_||_| |___\___/ ##
############################################################ Set IO Pins */
// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
//const int ESC_PIN [2] = {17, 16}; // ESC_L, ESC_R
const int ESC_L_PIN = 17;
const int ESC_R_PIN = 16;

//const int Servo_PIN [4] = {32, 33, 25, 26}; // x_L, y_L, x_R, y_R
const int Servo_x_L_PIN = 32;
const int Servo_y_L_PIN = 33;
const int Servo_x_R_PIN = 25;
const int Servo_y_R_PIN = 26;

/*########################################
##  ___      _ _    __   __            ##
## |_ _|_ _ (_) |_  \ \ / /_ _ _ _    ##
##  | || ' \| |  _|  \ V / _` | '_|  ##
## |___|_||_|_|\__|   \_/\__,_|_|   ##
################################################## Initialize variables */
bool ARMED = false;
//bool ARMED_prev = false;
int speed = 0;
int v_x = 0;
int v_y = 0;
int omega = 0;

int loopCount = 0;

/*####################################################
##  ___      _ _     ___          _               ##
## |_ _|_ _ (_) |_  |   \ _____ _(_)__ ___ ___   ##
##  | || ' \| |  _| | |) / -_) V / / _/ -_|_-<  ##
## |___|_||_|_|\__| |___/\___|\_/|_\__\___/__/ ##
#################################################### Initialize Devices */

/*############
## Sensors ##
########################################################################*/

// Gyro ------------------------------------------------------------------
Adafruit_MPU6050 mpu;

/*
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
*/


/*#################
## Drive System ##
########################################################################*/

// Gimbals --------------------------------------------------------------
// hold the L1 and/or R1 button and the D-Pad to adjust the trim for the x and y directions.
int Gimbal_L_Offset[2] = {-60, 100};  // y, x
int Gimbal_R_Offset[2] = {60, -100};  // y, x

int Trim_range[2] = {-500, 500};  // min, max


// ESCs ------------------------------------------------------------------
Servo ESC_L;
Servo ESC_R;

const int ESC_min = 1000;
const int ESC_set = 1250;
const int ESC_max = 1500;



int ESC_output = ESC_min;
//int ESC_L_output = ESC_min;
//int ESC_R_output = ESC_min;

// Servos ----------------------------------------------------------------
Servo servo_x_L;
Servo servo_y_L;
Servo servo_x_R;
Servo servo_y_R;

const int Servo_min = 1000;
const int Servo_mid = 1500;
const int Servo_max = 2000;

int Servo_x_L_output = Servo_mid;
int Servo_y_L_output = Servo_mid;
int Servo_x_R_output = Servo_mid;
int Servo_y_R_output = Servo_mid;

ESP32PWM pwm;


/*###################
## PS4 Controller ##
########################################################################*/
unsigned long debounceDelay = 50;    // debounce time
// LED -------------------------------------------------------------------
int r = 255;
int g = 0;
int b = 0;

// Inputs ----------------------------------------------------------------

// Sticks
int LSXinput = 0;
int LSYinput = 0;
int RSXinput = 0;
int RSYinput = 0;
int LSXrange [2] = {-127, 127};
int LSYrange [2] = {-100, 75};  // BROKEN
int RSXrange [2] = {-127, 127};
int RSYrange [2] = {-127, 127};

// Triggers
int L2input = 0;
int R2input = 0;
int L2range [2] = {0, 255};
int R2range [2] = {0, 255};






/*##############
## Bluetooth ##
########################################################################*/
/*
bool initBluetooth() {
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
}
 
void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02X", (int)point[i]);
    Serial.print(str);
 
    if (i < 5){
      Serial.print(":");
    }
  }
}
*/

/*###########################################
##  ___             _   _                 ##
## | __|  _ _ _  __| |_(_)___ _ _  ___   ##
## | _| || | ' \/ _|  _| / _ \ ' \(_-<  ##
## |_| \_,_|_||_\__|\__|_\___/_||_/__/ ##
############################################################# Functions */

/*############
## Sensors ##
########################################################################*/

// Gyro ------------------------------------------------------------------
void gyro_setup(){
  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");
  /*
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
  */
}

void gyro_loop(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Accelerometer ");
  Serial.print("X: ");
  Serial.print(a.acceleration.x, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Y: ");
  Serial.print(a.acceleration.y, 1);
  Serial.print(" m/s^2, ");
  Serial.print("Z: ");
  Serial.print(a.acceleration.z, 1);
  Serial.println(" m/s^2");

  Serial.print("Gyroscope ");
  Serial.print("X: ");
  Serial.print(g.gyro.x, 1);
  Serial.print(" rps, ");
  Serial.print("Y: ");
  Serial.print(g.gyro.y, 1);
  Serial.print(" rps, ");
  Serial.print("Z: ");
  Serial.print(g.gyro.z, 1);
  Serial.println(" rps");
  delay(period);
  /*
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value

  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.0; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 0.0; // AccErrorY ~(-1.58)

  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  GyroX = GyroX + 8.24; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 0.31; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.11; // GyroErrorZ ~ (-0.8)

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
  Serial.print(roll);
  Serial.print("/");
  Serial.print(pitch);
  Serial.print("/");
  Serial.println(yaw);
  */
}
/*
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
*/
/*###################
## PS4 Controller ##
########################################################################*/
// Calculates the next value in a rainbow sequence
void nextRainbowColor() {
  if (r > 0 && b == 0) { r--; g++; }
  if (g > 0 && r == 0) { g--; b++; }
  if (b > 0 && g == 0) { r++; b--; }
}



// Buttons
//https://github.com/kimballa/button-debounce/blob/main/examples/pressAndHold/pressAndHold.ino
static void ARM_buttonHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    switch(btnId) {
      case 4:
        ARMED = true;
        PS4.setLed(255, 0, 0);
        PS4.sendToController();
        break;
      case 5:
        ARMED = false;
        PS4.setLed(0, 255, 0);
        PS4.sendToController();
        break;
    }
  } else {
    // btnState == BTN_OPEN.
    if (DEBUG_ARM) Serial.println("Released button");
  }
}


static void Trim_buttonHandler(uint8_t btnId, uint8_t btnState) {
  int us_step = 10;
  if (btnState == BTN_PRESSED) {
    switch(btnId) {
      case 0:
        if (PS4.L1() && Gimbal_L_Offset[0] < Trim_range[1]) {
          Gimbal_L_Offset[1] -= us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] < Trim_range[1]) {
          Gimbal_R_Offset[1] += us_step;
        }
        break;
      case 1:
        if (PS4.L1() && Gimbal_L_Offset[0] > Trim_range[0]) {
          Gimbal_L_Offset[1] += us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] > Trim_range[0]) {
          Gimbal_R_Offset[1] -= us_step;
        }
        break;
      case 2:
        if (PS4.L1() && Gimbal_L_Offset[1] > Trim_range[0]) {
          Gimbal_L_Offset[0] += us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] > Trim_range[0]) {
          Gimbal_R_Offset[0] += us_step;
        }
        break;
      case 3:
        if (PS4.L1() && Gimbal_L_Offset[1] < Trim_range[1]) {
          Gimbal_L_Offset[0] -= us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] < Trim_range[1]) {
          Gimbal_R_Offset[0] -= us_step;
        }
        break;
    }
    if (DEBUG_Trim) Serial.println("Trim Adjusted");
  } else {
    // btnState == BTN_OPEN.
    if (DEBUG_Trim) Serial.println("Released button");
  }
}
// Define your button with a unique id (0) and handler function.
// (The ids are so one handler function can tell different buttons apart if necessary.)
static Button UpButton(0, Trim_buttonHandler);
static Button DownButton(1, Trim_buttonHandler);
static Button LeftButton(2, Trim_buttonHandler);
static Button RightButton(3, Trim_buttonHandler);

static Button L3Button(4, ARM_buttonHandler);
static Button R3Button(5, ARM_buttonHandler);

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  UpButton.update(PS4.Up());
  DownButton.update(PS4.Down());
  LeftButton.update(PS4.Left());
  RightButton.update(PS4.Right());
  L3Button.update(PS4.L3());
  R3Button.update(PS4.R3());
}

static void pollAnalog(){
  LSXinput = PS4.LStickX();
  LSYinput = PS4.LStickY();
  RSXinput = PS4.RStickX();
  RSYinput = PS4.RStickY();

  L2input = PS4.L2Value();
  R2input = PS4.R2Value();
}




/*#################
## Drive System ##
########################################################################*/

void setSpeed() {
  if (PS4.R2()) {
    speed = map(PS4.R2Value(), R2range[0], R2range[1], ESC_min, ESC_max);
    speed = constrain(speed, ESC_min, ESC_max);
    if (ESC_output < speed) ESC_output = speed;
  }
  if (PS4.L2()) {
    speed = map(PS4.L2Value(), L2range[0], L2range[1], ESC_max, ESC_min);
    speed = constrain(speed, ESC_min, ESC_max);
    if (ESC_output > speed) ESC_output = speed;
  }

  ESC_L.write(ESC_output);
  ESC_R.write(ESC_output);
}


void setServo_us(int id, int us) {
  us = constrain(us, 1000, 2000);
  switch (id) {
    case 0:
      servo_x_L.write(us);
      break;
    case 1:
      servo_y_L.write(us);
      break;
    case 2:
      servo_x_R.write(us);
      break;
    case 3:
      servo_y_R.write(us);
      break;
  }
} 

void setVector() {

  v_x = RSXinput;
  v_y = RSYinput;
  omega = map(LSXinput, LSXrange[0], LSXrange[1], -500, 500);

  Servo_x_L_output = map(v_x * 0.5 , RSXrange[0], RSXrange[1], Servo_min, Servo_max);
  Servo_y_L_output = map(-v_y * 1, RSYrange[0], RSYrange[1], Servo_min, Servo_max);
  Servo_x_R_output = map(v_x * 0.5, RSXrange[0], RSXrange[1], Servo_min, Servo_max);
  Servo_y_R_output = map(v_y * 1, RSYrange[0], RSYrange[1], Servo_min, Servo_max);

  setServo_us(0, Servo_x_L_output + Gimbal_L_Offset[1]);
  setServo_us(1, Servo_y_L_output + Gimbal_L_Offset[0]);
  setServo_us(2, Servo_x_R_output + Gimbal_R_Offset[1]);
  setServo_us(3, Servo_y_R_output + Gimbal_R_Offset[0]);

  if (ARMED) {
    ESC_L.write(ESC_output + omega);
    ESC_R.write(ESC_output - omega);
  }

}

/*###################
## DEBUG & REPORT ##
########################################################################*/
void DEBUG_REPORT(){
  if(millis() >= time_now + period){
    time_now += period;

    //Serial.print("BT MAC Address: ");
    //printDeviceAddress();
    //Serial.println();
    Serial.print("===================== ");

    if (PS4.isConnected()) {
      Serial.println("Connected!");
      // Active Buttons -----------------------------------------------------
      if (PS4.L1()) Serial.println("L1 Button");
      if (PS4.R1()) Serial.println("R1 Button");
      if (PS4.L3()) Serial.println("L3 Button");
      if (PS4.R3()) Serial.println("R3 Button");

      if (PS4.Right()) Serial.println("Right Button");
      if (PS4.Down()) Serial.println("Down Button");
      if (PS4.Up()) Serial.println("Up Button");
      if (PS4.Left()) Serial.println("Left Button");

      Serial.println("---------------------");
       // Variables ----------------------------------------------------------
      if (REPORT_Offset) {
        Serial.print("Offset: ");
        Serial.print("Left: ");
        Serial.print(Gimbal_L_Offset[0]);
        Serial.print(", ");
        Serial.print(Gimbal_L_Offset[1]);
        Serial.println("  // y, x");

        Serial.print("Right: ");
        Serial.print(Gimbal_R_Offset[0]);
        Serial.print(", ");
        Serial.print(Gimbal_R_Offset[1]);
        Serial.println("  // y, x");
      } 
      Serial.println("---------------------");
      if (REPORT_Sticks) {
        Serial.print("LSX: ");
        Serial.println(LSXinput);
        Serial.print("LSY: ");
        Serial.println(LSYinput);
        Serial.print("RSX: ");
        Serial.println(RSXinput);
        Serial.print("RSY: ");
        Serial.println(RSYinput);
      }
      if (REPORT_triggers) {
        Serial.print("L2: ");
        Serial.println(L2input);
        Serial.print("R2: ");
        Serial.println(R2input);
      }
      if (REPORT_ESC) {
        Serial.print("ESC_output: ");
        Serial.println(ESC_output);
      }
      if (DEBUG_Servo) {
        Serial.print("Servo_x_L: ");
        Serial.println(Servo_x_L_output);
        Serial.print("Servo_y_L: ");
        Serial.println(Servo_y_L_output);
        Serial.print("Servo_x_R: ");
        Serial.println(Servo_x_R_output);
        Serial.print("Servo_y_R: ");
        Serial.println(Servo_y_R_output);
      }
      Serial.println("---------------------");
      Serial.printf("Battery Level: %d", PS4.Battery());
      if (PS4.Charging()) Serial.println(" (Charging)");
      else Serial.println();
    }
    else Serial.println("Connecting...");
  }
}


/*###################################################
##   __  __      _        _                       ##
##  |  \/  |__ _(_)_ _   | |   ___  ___ _ __     ##
##  | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \   ##
##  |_|  |_\__,_|_|_||_| |____\___/\___/ .__/  ##
##                                     |_|    ##
########################################################################*/
/*#################
## Arduino Setup ##
########################################################################*/
void setup() {
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  ESC_L.setPeriodHertz(50);      // Standard 50hz servo
  ESC_R.setPeriodHertz(50);      // Standard 50hz servo
  servo_x_L.setPeriodHertz(50);      // Standard 50hz servo
	servo_y_L.setPeriodHertz(50);      // Standard 50hz servo
  servo_x_R.setPeriodHertz(50);      // Standard 50hz servo
  servo_y_R.setPeriodHertz(50);      // Standard 50hz servo


  ESC_L.attach(ESC_L_PIN, ESC_min, ESC_max);
  ESC_R.attach(ESC_R_PIN, ESC_min, ESC_max);
  //L3_R3_Button.setPushDebounceInterval(5000);

  servo_x_L.attach(Servo_x_L_PIN, Servo_min, Servo_max);
  servo_y_L.attach(Servo_y_L_PIN, Servo_min, Servo_max);
  servo_x_R.attach(Servo_x_R_PIN, Servo_min, Servo_max);
  servo_y_R.attach(Servo_y_R_PIN, Servo_min, Servo_max);



  Serial.begin(115200);
  PS4.begin("c0:38:96:b3:c5:c5");

  gyro_setup();



  //Serial.print("WiFi MAC Address:  ");
  //Serial.println(WiFi.macAddress());
  
  //initBluetooth();
  //Serial.print("BT MAC Address:  ");
  //printDeviceAddress();
  Serial.println("Ready.");
}

/*#################
## Arduino Loop ##
########################################################################*/
void loop() {
  //calculate_IMU_error();
  gyro_loop();
  if (PS4.isConnected()) {
    //Emergency Brake
    if (PS4.Circle()) {
      ESC_output = ESC_min;
      ESC_L.write(ESC_output);
      ESC_R.write(ESC_output);
    }
    pollButtons();
    pollAnalog();
    setVector();
    switch (ARMED) {
      case true:
        //pollButtons();
        setSpeed();
        
        break;
      case false:
        ESC_L.write(ESC_min);
        ESC_R.write(ESC_min);
        break;
    } 
  } 
  else {
    ESC_L.write(ESC_min);
    ESC_R.write(ESC_min);

    servo_x_L.write(Servo_mid);
    servo_y_L.write(Servo_mid);
    servo_x_R.write(Servo_mid);
    servo_y_R.write(Servo_mid);
  }
  DEBUG_REPORT();
}