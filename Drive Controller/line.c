const int sensor1Pin = 25; // Connect sensor 1 to analog pin A0
const int sensor2Pin = 26; // Connect sensor 2 to analog pin A1
const int sensor3Pin = 27; // Connect sensor 3 to analog pin A2
const int sensor4Pin = 34; // Connect sensor 4 to analog pin A3
const int sensor5Pin = 35; // Connect sensor 5 to analog pin A4

void setup() {
  Serial.begin(9600);
}

void loop() {
  int s1value = analogRead(sensor1Pin);
  int s2value = analogRead(sensor2Pin);
  int s3value = analogRead(sensor3Pin);
  int s4value = analogRead(sensor4Pin);
  int s5value = analogRead(sensor5Pin);

  if (s3value < s1value && s3value < s2value && s3value < s4value && s3value < s5value) {
    Serial.println("Centered to the line");
  } else if (s1value < s3value && s1value < s4value && s1value < s5value && 
    s2value < s3value && s2value < s4value && s2value < s5value) {    Serial.println("Too right");
  } else if (s4value < s3value && s4value < s2value && s4value < s1value &&
    s5value < s3value && s5value < s2value && s5value < s1value) {
    Serial.println("Too left");
  } else if (s1value > s2value) {
    Serial.println("little right");
  } else if (s5value > s4value) {
    Serial.println("little left");
  }
  delay(2000); // 1 measurement per 2 seconds
}

