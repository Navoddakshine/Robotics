int Sensor  =  10;  // Declaration of the sensor input spin
   
void setup  ( )
{
  Serial.begin (9600)  ;  // Initialization  serial output
  pinMode (Sensor, INPUT)   ;  //  Initialization Sensorpin
}
  
//  The program reads the current state of the sensor pin and
// displays in the serial console whether the line tracker is currently 
// on the line or not
void loop  ( )
{
  bool val =  digitalRead (sensor)   ;  //  The current signal on the sensor is read
  
  if   (val  = =  HIGH )  //  If a signal has been detected, the LED is switched on.
  {
    Serial.println (“LineTracker is above the line”);
  }
  else
  {
    Serial.println (“Linetracker is out of line”);
  }
  Serial.println (“------------------------------------”)  ;
  delay (500)  ;  // Pasuse between the measurement of 500ms
}
