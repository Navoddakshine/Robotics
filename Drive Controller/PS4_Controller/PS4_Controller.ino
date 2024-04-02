/*
TEST


https://techtutorialsx.com/2018/03/09/esp32-arduino-getting-the-bluetooth-device-address/
https://github.com/pablomarquez76/PS4_Controller_Host/tree/main/examples

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
//#include <analogWrite.h>
#include <ESP32Servo.h>

/*##################################
##  ___      _                   ##
## |   \ ___| |__ _  _ __ _     ##
## | |) / -_) '_ \ || / _` |   ##
## |___/\___|_.__/\_,_\__, |  ##
##                    |___/  ##
############################################################ DEBUG MODES */
#define DEBUG_sticks 1
#define DEBUG_triggers 1
#define DEBUG_bumpers 0
#define DEBUG_ESC 1
#define DEBUG_Servo 1
#define DEBUG_Offset 0

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


/*#########################################
##  ___      _ _    __   __            ##
## |_ _|_ _ (_) |_  \ \ / /_ _ _ _    ##
##  | || ' \| |  _|  \ V / _` | '_|  ##
## |___|_||_|_|\__|   \_/\__,_|_|   ##
################################################## Initialize variables */
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
// ESCs ------------------------------------------------------------------
Servo ESC_L;
Servo ESC_R;

const int ESC_min = 1000;
const int ESC_max = 1200;

int ESC_L_output = ESC_min;
int ESC_R_output = ESC_min;

// Servos ----------------------------------------------------------------
Servo servo_x_L;
Servo servo_y_L;
Servo servo_x_R;
Servo servo_y_R;

const int Servo_min = 1300;
const int Servo_mid = 1500;
const int Servo_max = 1700;

//int Servo_Offset [4] = {0, 0, 0, 0};
int Servo_Offset_x_L = 0;
int Servo_Offset_y_L = 0;
int Servo_Offset_x_R = 0;
int Servo_Offset_y_R = 0;


int Servo_x_L_output = Servo_mid;
int Servo_y_L_output = Servo_mid;
int Servo_x_R_output = Servo_mid;
int Servo_y_R_output = Servo_mid;

ESP32PWM pwm;


/*###################
## PS4 Controller ##
########################################################################*/
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

// Bumpers
bool L1input = false;
bool R1input = false;

// D-Pad
bool Upinput = false;
bool Downinput = false;
bool Leftinput = false;
bool Rightinput = false;


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

void PS4_sendData() {
    // Sets the color of the controller's front light
    // Params: Red, Green,and Blue
    // See here for details: https://www.w3schools.com/colors/colors_rgb.asp
    PS4.setLed(r, g, b);
    nextRainbowColor();

    // Sets how fast the controller's front light flashes
    // Params: How long the light is on , how long the light is off
    // Range: 0->255 (255 = 2550ms), Set to 0, 0 for the light to remain on
    PS4.setFlashRate(PS4.LStickY(), PS4.RStickY());

    // Sets the rumble of the controllers
    // Params: Weak rumble intensity, Strong rumble intensity
    // Range: 0->255
    PS4.setRumble(PS4.L2Value(), PS4.R2Value());

    // Sends data set in the above three instructions to the controller
    PS4.sendToController();

    // Don't send data to the controller immediately, will cause buffer overflow
    delay(10);
}

void PS4_recieveData() {
  // Below has all accessible outputs from the controller
    if (PS4.Right()) Serial.println("Right Button");
    if (PS4.Down()) Serial.println("Down Button");
    if (PS4.Up()) Serial.println("Up Button");
    if (PS4.Left()) Serial.println("Left Button");

    if (PS4.Square()) Serial.println("Square Button");
    if (PS4.Cross()) Serial.println("Cross Button");
    if (PS4.Circle()) Serial.println("Circle Button");
    if (PS4.Triangle()) Serial.println("Triangle Button");

    if (PS4.UpRight()) Serial.println("Up Right");
    if (PS4.DownRight()) Serial.println("Down Right");
    if (PS4.UpLeft()) Serial.println("Up Left");
    if (PS4.DownLeft()) Serial.println("Down Left");

    if (PS4.L1()) Serial.println("L1 Button");
    if (PS4.R1()) Serial.println("R1 Button");

    if (PS4.Share()) Serial.println("Share Button");
    if (PS4.Options()) Serial.println("Options Button");
    if (PS4.L3()) Serial.println("L3 Button");
    if (PS4.R3()) Serial.println("R3 Button");

    if (PS4.PSButton()) Serial.println("PS Button");
    if (PS4.Touchpad()) Serial.println("Touch Pad Button");

    if (PS4.L2()) {
      Serial.printf("L2 button at %d\n", PS4.L2Value());
    }
    if (PS4.R2()) {
      Serial.printf("R2 button at %d\n", PS4.R2Value());
    }

    if (PS4.LStickX()) {
      Serial.printf("Left Stick x at %d\n", PS4.LStickX());
    }
    if (PS4.LStickY()) {
      Serial.printf("Left Stick y at %d\n", PS4.LStickY());
    }
    if (PS4.RStickX()) {
      Serial.printf("Right Stick x at %d\n", PS4.RStickX());
    }
    if (PS4.RStickY()) {
      Serial.printf("Right Stick y at %d\n", PS4.RStickY());
    }

    if (PS4.Charging()) Serial.println("The controller is charging");
    if (PS4.Audio()) Serial.println("The controller has headphones attached");
    if (PS4.Mic()) Serial.println("The controller has a mic attached");

    Serial.printf("Battery Level : %d\n", PS4.Battery());
    Serial.println();
    delay(1000);
}


void PS4_getControlInput(){
    //PS4_sendData();
    //PS4_recieveData();

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
void setOffset(){
  if (PS4.L1()) {
      //Serial.println("L1");
      if (PS4.UpRight()) {
        Serial.println("UpRight");
        Servo_Offset_x_L = Servo_Offset_x_L + 10;
      }
      if (PS4.DownRight()) {
        Serial.println("DownRight");
        Servo_Offset_x_L = Servo_Offset_x_L - 10;
      }
      if (PS4.UpLeft()) {
        Serial.println("UpLeft");
        Servo_Offset_y_L = Servo_Offset_y_L + 10;
      }
      if (PS4.DownLeft()) {
        Serial.println("DownLeft");
        Servo_Offset_y_L = Servo_Offset_y_L - 10;
      }
    }


    if (PS4.R1()) {
      //Serial.println("R1");
      if (PS4.UpRight()) {
        Serial.println("UpRight");
        Servo_Offset_x_R = Servo_Offset_x_R + 10;
      }
      if (PS4.DownRight()) {
        Serial.println("DownRight");
        Servo_Offset_x_R = Servo_Offset_x_R - 10;
      }
      if (PS4.UpLeft()) {
        Serial.println("UpLeft");
        Servo_Offset_y_R = Servo_Offset_y_R + 10;
      }
      if (PS4.DownLeft()) {
        Serial.println("DownLeft");
        Servo_Offset_y_R = Servo_Offset_y_R - 10;
      }
    }
}

void setECS_us(int id, int us) {
  us = constrain(us, ESC_min, ESC_max);
  switch (id) {
    case 0:
      ESC_L.write(us);
      break;
    case 1:
      ESC_R.write(us);
      break;
  }
}

void setSpeed() {
  if (PS4.R2()) {
      speed = speed + PS4.R2Value();
    }
  if (PS4.L2()) {
      speed = speed - PS4.L2Value();
    }  
  setECS_us(0, speed);
  setECS_us(1, speed);
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

  setServo_us(0, Servo_x_L_output + Servo_Offset_x_L);
  setServo_us(1, Servo_y_L_output + Servo_Offset_y_L);
  setServo_us(2, Servo_x_R_output + Servo_Offset_x_R);
  setServo_us(3, Servo_y_R_output + Servo_Offset_y_R);

}

/*##########
## DEBUG ##
########################################################################*/
void DEBUGreport(){
  if(millis() >= time_now + period){
    time_now += period;
    
    if (DEBUG_sticks) {
      Serial.print("LSX: ");
      Serial.println(LSXinput);
      Serial.print("LSY: ");
      Serial.println(LSYinput);
      Serial.print("RSX: ");
      Serial.println(RSXinput);
      Serial.print("RSY: ");
      Serial.println(RSYinput);
    }
    if (DEBUG_triggers) {
      Serial.print("L2: ");
      Serial.println(L2input);
      Serial.print("R2: ");
      Serial.println(R2input);
    }
    if (DEBUG_bumpers) {
      Serial.print("L1: ");
      Serial.println(PS4.L1());
      Serial.print("R1: ");
      Serial.println(PS4.R1());
    }

    if (DEBUG_ESC) {
      Serial.print("ESC_L: ");
      Serial.println(ESC_L_output);
      Serial.print("ESC_R: ");
      Serial.println(ESC_R_output);
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

    if (DEBUG_Offset) {
      Serial.print("Servo_Offset_x_L: ");
      Serial.println(Servo_Offset_x_L);
      Serial.print("Servo_Offset_y_L: ");
      Serial.println(Servo_Offset_y_L);
      Serial.print("Servo_Offset_x_R: ");
      Serial.println(Servo_Offset_x_R);
      Serial.print("Servo_Offset_y_R: ");
      Serial.println(Servo_Offset_y_R);
    } 

    if (PS4.Charging()) Serial.println("The controller is charging");
    Serial.printf("Controller Battery Level : %d\n", PS4.Battery());
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

  servo_x_L.attach(Servo_x_L_PIN, Servo_min, Servo_max);
  servo_y_L.attach(Servo_y_L_PIN, Servo_min, Servo_max);
  servo_x_R.attach(Servo_x_R_PIN, Servo_min, Servo_max);
  servo_y_R.attach(Servo_y_R_PIN, Servo_min, Servo_max);



  Serial.begin(115200);
  PS4.begin();
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
    PS4_getControlInput();
    //setOffset();
    setSpeed();
    setVector();
  } else {
    ESC_L.write(ESC_min);
    ESC_R.write(ESC_min);

    servo_x_L.write(Servo_mid);
    servo_y_L.write(Servo_mid);
    servo_x_R.write(Servo_mid);
    servo_y_R.write(Servo_mid);
  }
  DEBUGreport();
}