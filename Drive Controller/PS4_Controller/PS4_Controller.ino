/*
TEST


https://techtutorialsx.com/2018/03/09/esp32-arduino-getting-the-bluetooth-device-address/
https://github.com/pablomarquez76/PS4_Controller_Host/tree/main/examples

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
bool ARMED_prev = false;
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

int Trim_range[2] = {-500, 500};  // min, max


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

const int Servo_min = 1250;
const int Servo_mid = 1500;
const int Servo_max = 1750;

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
          Gimbal_L_Offset[0] += us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] < Trim_range[1]) {
          Gimbal_R_Offset[0] += us_step;
        }
        break;
      case 1:
        if (PS4.L1() && Gimbal_L_Offset[0] > Trim_range[0]) {
          Gimbal_L_Offset[0] -= us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] > Trim_range[0]) {
          Gimbal_R_Offset[0] -= us_step;
        }
        break;
      case 2:
        if (PS4.L1() && Gimbal_L_Offset[1] > Trim_range[0]) {
          Gimbal_L_Offset[1] -= us_step;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] > Trim_range[0]) {
          Gimbal_R_Offset[1] -= us_step;
        }
        break;
      case 3:
        if (PS4.L1() && Gimbal_L_Offset[1] < Trim_range[1]) {
          Gimbal_L_Offset[1] += 10;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] < Trim_range[1]) {
          Gimbal_R_Offset[1] += 10;
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
  omega = LSXinput;

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