/*
https://techtutorialsx.com/2018/03/09/esp32-arduino-getting-the-bluetooth-device-address/
https://github.com/pablomarquez76/PS4_Controller_Host/tree/main/examples
https://github.com/rodneybakiskan/ps5-esp32
https://github.com/yesbotics/dualsense-controller-arduino/tree/main
https://github.com/kimballa/button-debounce/blob/main/examples/buttons/buttons.ino

https://github.com/jkb-git/ESP32Servo/blob/master/examples/Multiple-Servo-Example-Arduino/Multiple-Servo-Example-Arduino.ino


MAC-Address:  58:10:31:A0:AB:95
MAC-Address:  34:7d:f6:EA:4A:0B

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
#include <ps5Controller.h>
#include <debounce.h>
//#include <analogWrite.h>
#include <ESP32Servo.h>

/*##################################
##  ___      _                   ##
## |   \ ___| |__ _  _ __ _     ##
## | |) / -_) '_ \ || / _` |   ##
## |___/\___|_.__/\_,_\__, |  ##
##                    |___/  ##
############################################################ DEBUG MODES */
#define DEBUG_Servo 0

#define DEBUG_Trim 0

#define REPORT_ESC 1
#define REPORT_Offset 1

#define REPORT_Sticks 0
#define REPORT_triggers 1


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
const int Servo_x_L_PIN = 33;
const int Servo_y_L_PIN = 32;
const int Servo_x_R_PIN = 26;
const int Servo_y_R_PIN = 25;


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

/*#################
## Drive System ##
########################################################################*/

// Gimbals --------------------------------------------------------------
// hold the L1 and/or R1 button and the D-Pad to adjust the trim for the x and y directions.
int Gimbal_L_Offset[2] = {0, 0};  // y, x
int Gimbal_R_Offset[2] = {0, 0};  // y, x

int Trim_range[2] = {-200, 200};  // min, max


// ESCs ------------------------------------------------------------------
Servo ESC_L;
Servo ESC_R;

const int ESC_min = 1000;
const int ESC_set = 1500;
const int ESC_max = 1800;



int ESC_output = ESC_min;
//int ESC_L_output = ESC_min;
//int ESC_R_output = ESC_min;

// Servos ----------------------------------------------------------------
Servo servo_x_L;
Servo servo_y_L;
Servo servo_x_R;
Servo servo_y_R;

const int Servo_min = 1300;
const int Servo_mid = 1500;
const int Servo_max = 1700;

int Servo_x_L_output = Servo_mid;
int Servo_y_L_output = Servo_mid;
int Servo_x_R_output = Servo_mid;
int Servo_y_R_output = Servo_mid;

ESP32PWM pwm;


/*###################
## ps5 Controller ##
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
/*###################
## ps5 Controller ##
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
    return; // Do nothing on press-down.
  }
  else {
  // btnState == BTN_OPEN.
    ARMED = true;
    ps5.setLed(255, 0, 0);
    ps5.sendToController();
    Serial.println("Armed.");
  }
}


static void Trim_buttonHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    switch(btnId) {
      case 0:
        if (ps5.L1() && Gimbal_L_Offset[0] < Trim_range[1]) {
          Gimbal_L_Offset[0] += 1;
        }
        if (ps5.R1() && Gimbal_R_Offset[0] < Trim_range[1]) {
          Gimbal_R_Offset[0] += 1;
        }
        break;
      case 1:
        if (ps5.L1() && Gimbal_L_Offset[0] > Trim_range[0]) {
          Gimbal_L_Offset[0] -= 1;
        }
        if (ps5.R1() && Gimbal_R_Offset[0] > Trim_range[0]) {
          Gimbal_R_Offset[0] -= 1;
        }
        break;
      case 2:
        if (ps5.L1() && Gimbal_L_Offset[1] > Trim_range[0]) {
          Gimbal_L_Offset[1] -= 1;
        }
        if (ps5.R1() && Gimbal_R_Offset[1] > Trim_range[0]) {
          Gimbal_R_Offset[1] -= 1;
        }
        break;
      case 3:
        if (ps5.L1() && Gimbal_L_Offset[1] < Trim_range[1]) {
          Gimbal_L_Offset[1] += 1;
        }
        if (ps5.R1() && Gimbal_R_Offset[1] < Trim_range[1]) {
          Gimbal_R_Offset[1] += 1;
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

static Button L3_R3_Button(4, ARM_buttonHandler);

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  UpButton.update(ps5.Up());
  DownButton.update(ps5.Down());
  LeftButton.update(ps5.Left());
  RightButton.update(ps5.Right());

  //bool L3_R3_input = ps5.L3() && ps5.R3();
  //L3_R3_Button.update(L3_R3_input);
}

static void pollAnalog(){
  LSXinput = ps5.LStickX();
  LSYinput = ps5.LStickY();
  RSXinput = ps5.RStickX();
  RSYinput = ps5.RStickY();

  L2input = ps5.L2Value();
  R2input = ps5.R2Value();
}




/*#################
## Drive System ##
########################################################################*/

void setSpeed() {
  if (ps5.R2()) {
    speed = map(ps5.R2Value(), R2range[0], R2range[1], ESC_min, ESC_max);
    speed = constrain(speed, ESC_min, ESC_max);
    if (ESC_output < speed) ESC_output = speed;
  }
  if (ps5.L2()) {
    speed = map(ps5.L2Value(), L2range[0], L2range[1], ESC_max, ESC_min);
    speed = constrain(speed, ESC_min, ESC_max);
    if (ESC_output > speed) ESC_output = speed;
  }
  ESC_L.write(ESC_output);
  ESC_R.write(ESC_output);
}


void setServo_us(int id, int us) {
  us = constrain(us, Servo_min, Servo_max);
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

  Servo_x_L_output = map(v_x, RSXrange[0], RSXrange[1], Servo_min, Servo_max);
  Servo_y_L_output = map(-v_y, RSYrange[0], RSYrange[1], Servo_min, Servo_max);
  Servo_x_R_output = map(v_x, RSXrange[0], RSXrange[1], Servo_min, Servo_max);
  Servo_y_R_output = map(v_y, RSYrange[0], RSYrange[1], Servo_min, Servo_max);

  setServo_us(0, Servo_x_L_output + Gimbal_L_Offset[1]);
  setServo_us(1, Servo_y_L_output + Gimbal_L_Offset[0]);
  setServo_us(2, Servo_x_R_output + Gimbal_R_Offset[1]);
  setServo_us(3, Servo_y_R_output + Gimbal_R_Offset[0]);

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

    if (ps5.isConnected()) {
      Serial.println("Connected!");
      // Active Buttons -----------------------------------------------------
      if (ps5.L1()) Serial.println("L1 Button");
      if (ps5.R1()) Serial.println("R1 Button");
      if (ps5.L3()) Serial.println("L3 Button");
      if (ps5.R3()) Serial.println("R3 Button");

      if (ps5.Right()) Serial.println("Right Button");
      if (ps5.Down()) Serial.println("Down Button");
      if (ps5.Up()) Serial.println("Up Button");
      if (ps5.Left()) Serial.println("Left Button");

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
      Serial.printf("Battery Level: %d", ps5.Battery());
      if (ps5.Charging()) Serial.println(" (Charging)");
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
  L3_R3_Button.setPushDebounceInterval(5000);

  servo_x_L.attach(Servo_x_L_PIN, Servo_min, Servo_max);
  servo_y_L.attach(Servo_y_L_PIN, Servo_min, Servo_max);
  servo_x_R.attach(Servo_x_R_PIN, Servo_min, Servo_max);
  servo_y_R.attach(Servo_y_R_PIN, Servo_min, Servo_max);



  Serial.begin(115200);
  ps5.begin("58:10:31:A0:AB:95");



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
  if (ps5.isConnected()) {
    //Emergency Brake
    if (ps5.Circle()) {
      ESC_output = ESC_min;
      ESC_L.write(ESC_output);
      ESC_R.write(ESC_output);
      ARMED = false;
      Serial.println("Disarmed.");
      ps5.setLed(0, 255, 0);
      ps5.sendToController();
      delay (10);
    }
    pollButtons();
    switch (ARMED) {
      case true:
        //pollButtons();
        pollAnalog();
        setSpeed();
        setVector();
        break;
      case false:
        bool L3_R3_input = ps5.L3() && ps5.R3();
        L3_R3_Button.update(L3_R3_input);
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