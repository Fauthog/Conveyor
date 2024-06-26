// Send two integer numbers indicating angle values of the servo. The first number is 
// for the big servo and the second number for the small servo, e.g. 40,50.
//
// According to the library, the angle range is from 0-180. To get finer control over the angle
// one could use servo.writeMicroseconds() instead of servo.write(). See 
// https://github.com/arduino-libraries/Servo/blob/master/docs/api.md

#include <Servo.h> // By Arduino, v1.2.1

// Define servo objects
Servo big_servo;
Servo small_servo;

// Define servo pins
const int bigServoPin = 9;
const int smallServoPin = 10;

// Define the maximum length of the input string
const int BUFFER_SIZE = 64;

// Define the delimiter character
const char DELIMITER = ',';

// Create a buffer to store the incoming serial data
char buffer[BUFFER_SIZE];

// Variables to store the extracted float values
int bigServoAngle = 500;
int smallServoAngle = 500;
int solenoid = 0;

// Flag to indicate whether a complete set of values has been received
bool newData = false;

void setup() {
  Serial.begin(9600);
  
  // Attach servos to the defined pins
  big_servo.attach(bigServoPin, 970, 2450); // 550, 2400 custom range for DS51150-12V servo
  small_servo.attach(smallServoPin, 500, 1150);

  Serial.print("Big servo angle: ");
  Serial.println(bigServoAngle);
  Serial.print("Small servo angle: ");
  Serial.println(smallServoAngle);

  pinMode(30, OUTPUT);    // sets the digital pin 13 as output
  digitalWrite(30, LOW);
}

void loop() {
  // Read serial data into the buffer
  readSerialData();  

  // If a complete set of values has been received, parse the data
  if (newData) {
    parseSerialData();
    // Print the parsed values
    Serial.print("Big servo angle: ");
    Serial.println(bigServoAngle);
    Serial.print("Small servo angle: ");
    Serial.println(smallServoAngle);
    
    // Reset the flag
    newData = false;
  }

  // Use the values to control the servos
  // big_servo.write(bigServoAngle);
  // small_servo.write(smallServoAngle);

  big_servo.writeMicroseconds(bigServoAngle);
  small_servo.writeMicroseconds(smallServoAngle);
  if (solenoid != 0) {
    digitalWrite(30, HIGH); // sets the digital pin 13 on
  }
  else {
     digitalWrite(30, LOW);  // sets the digital pin 13 off
  }

  // Small delay to debounce input readings
  delay(50);
}

void readSerialData() {
  static byte index = 0;
  while (Serial.available() > 0 && !newData) {
    char receivedChar = Serial.read();
    if (receivedChar != '\n') {
      buffer[index] = receivedChar;
      index++;
      if (index >= BUFFER_SIZE) {
        index = BUFFER_SIZE - 1;
      }
    } else {
      buffer[index] = '\0'; // Null terminate the string
      index = 0;
      newData = true;
    }
  }
}

void parseSerialData() {
  char *token = strtok(buffer, ",");
  if (token != NULL) {
    bigServoAngle = atoi(token); // Convert the token to a float
    token = strtok(NULL, ",");
    if (token != NULL) {
      smallServoAngle = atoi(token); // Convert the token to a float
      token = strtok(NULL, ",");
      if (token != NULL) {
        solenoid = atoi(token); // Convert the token to a float
    }
    }
   
  }
//  Serial.print("Solenoid: ");
//  Serial.println(solenoid);
}
