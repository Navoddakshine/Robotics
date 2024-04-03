/*
Gimbal Trim
Author: Dylan Nibourg
Date: 2024-04

Description:
This code is used to adjust the trim of the gimbals using a PS4 controller.
The trim is adjusted by holding the L1 or R1 button and then using the D-Pad to adjust the trim for the x and y directions.

The trim values can be reported on the serial monitor.
along with:
- the battery level of the PS4 controller
- whether it is charging
- the connection status of the PS4 controller.

The code uses
- PS4Controller library to interface with the PS4 controller.
- debounce library to debounce the buttons on the PS4 controller.
  - the Button class to handle the button debouncing.


Based on:
https://techtutorialsx.com/2018/03/09/esp32-arduino-getting-the-bluetooth-device-address/
https://github.com/pablomarquez76/PS4_Controller_Host/tree/main/examples
https://github.com/kimballa/button-debounce/blob/main/examples/buttons/buttons.ino
*/


/*###########################################
##  _    _ _                 _            ##
## | |  (_) |__ _ _ __ _ _ _(_)___ ___   ##
## | |__| | '_ \ '_/ _` | '_| / -_|_-<  ##
## |____|_|_.__/_| \__,_|_| |_\___/__/ ##
###################################################### Import libraries */
//#include "esp_bt_main.h"
//#include "esp_bt_device.h"
#include <PS4Controller.h>
#include<debounce.h>

/*##################################
##  ___      _                   ##
## |   \ ___| |__ _  _ __ _     ##
## | |) / -_) '_ \ || / _` |   ##
## |___/\___|_.__/\_,_\__, |  ##
##                    |___/  ##
############################################################ DEBUG MODES */
#define DEBUG_Trim 0
#define REPORT_Offset 1

int period = 1000;
unsigned long time_now = 0;


/*##############################
##  ___ _        ___ ___     ##
## | _ (_)_ _   |_ _/ _ \   ##
## |  _/ | ' \   | | (_) | ##
## |_| |_|_||_| |___\___/ ##
############################################################ Set IO Pins */

/*########################################
##  ___      _ _    __   __            ##
## |_ _|_ _ (_) |_  \ \ / /_ _ _ _    ##
##  | || ' \| |  _|  \ V / _` | '_|  ##
## |___|_||_|_|\__|   \_/\__,_|_|   ##
################################################## Initialize variables */
int Gimbal_L_Offset[2] = {0, 0};
int Gimbal_R_Offset[2] = {0, 0};

int Trim_range[2] = {-200, 200};


/*####################################################
##  ___      _ _     ___          _               ##
## |_ _|_ _ (_) |_  |   \ _____ _(_)__ ___ ___   ##
##  | || ' \| |  _| | |) / -_) V / / _/ -_|_-<  ##
## |___|_||_|_|\__| |___/\___|\_/|_\__\___/__/ ##
#################################################### Initialize Devices */

/*###################
## PS4 Controller ##
########################################################################*/
//unsigned long debounceDelay = 50;    // debounce time
// LED -------------------------------------------------------------------
int r = 255;
int g = 0;
int b = 0;



/*###########################################
##  ___             _   _                 ##
## | __|  _ _ _  __| |_(_)___ _ _  ___   ##
## | _| || | ' \/ _|  _| / _ \ ' \(_-<  ##
## |_| \_,_|_||_\__|\__|_\___/_||_/__/ ##
############################################################# Functions */
/*###################
## PS4 Controller ##
########################################################################*/

void nextRainbowColor() {
  if (r > 0 && b == 0) { r--; g++; }
  if (g > 0 && r == 0) { g--; b++; }
  if (b > 0 && g == 0) { r++; b--; }
}

static void Trim_buttonHandler(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    switch(btnId) {
      case 0:
        if (PS4.L1() && Gimbal_L_Offset[0] < Trim_range[1]) {
          Gimbal_L_Offset[0] += 1;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] < Trim_range[1]) {
          Gimbal_R_Offset[0] += 1;
        }
        break;
      case 1:
        if (PS4.L1() && Gimbal_L_Offset[0] > Trim_range[0]) {
          Gimbal_L_Offset[0] -= 1;
        }
        if (PS4.R1() && Gimbal_R_Offset[0] > Trim_range[0]) {
          Gimbal_R_Offset[0] -= 1;
        }
        break;
      case 2:
        if (PS4.L1() && Gimbal_L_Offset[1] > Trim_range[0]) {
          Gimbal_L_Offset[1] -= 1;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] > Trim_range[0]) {
          Gimbal_R_Offset[1] -= 1;
        }
        break;
      case 3:
        if (PS4.L1() && Gimbal_L_Offset[1] < Trim_range[1]) {
          Gimbal_L_Offset[1] += 1;
        }
        if (PS4.R1() && Gimbal_R_Offset[1] < Trim_range[1]) {
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

static void pollButtons() {
  // update() will call buttonHandler() if PIN transitions to a new state and stays there
  // for multiple reads over 25+ ms.
  UpButton.update(PS4.Up());
  DownButton.update(PS4.Down());
  LeftButton.update(PS4.Left());
  RightButton.update(PS4.Right());
}


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

      if (PS4.Right()) Serial.println("Right Button");
      if (PS4.Down()) Serial.println("Down Button");
      if (PS4.Up()) Serial.println("Up Button");
      if (PS4.Left()) Serial.println("Left Button");

      Serial.println("---------------------");
      // Variables ----------------------------------------------------------
      if (REPORT_Offset) {

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
  Serial.begin(115200);
  //PS4.begin("C0:38:96:B3:C5:C5");
  PS4.begin("c0:38:96:b3:c5:c5");
  //PS4.begin("C0:38:96:B3:C5:E6");
  //PS4.begin("c0:38:96:b3:c5:e6");

  //Serial.print("WiFi MAC Address:  ");
  //Serial.println(WiFi.macAddress());
  
  //initBluetooth();
  //Serial.print("BT MAC Address:  ");
  //printDeviceAddress();
  //Serial.println();
  Serial.println("Ready.");

}

/*#################
## Arduino Loop ##
########################################################################*/
void loop() {
  //PS4_testInputs();

  pollButtons();


  // DEBUG --------------------------------------------------------------
  DEBUG_REPORT();
}
