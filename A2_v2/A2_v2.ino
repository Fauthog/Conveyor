#include <Servo.h>

// Define servo objects
Servo big_servo;
Servo small_servo;

// Define input pins
const int inputPin1 = 2;
const int inputPin2 = 3;
const int inputPin3 = 4;

// Define servo pins
const int bigServoPin = 9;
const int smallServoPin = 10;

// Define servo angles for the input conditions
const int bigServoAngle0 = 970;   // big_servo angle for input (0,0)
const int bigServoAngle1 = 2300;  // big_servo angle for input (0,1)
const int bigServoAngle2 = 2450;  // big_servo angle for input (1,0) or (1,1)
const int smallServoAngleClose = 750;   // small_servo angle for input 0
const int smallServoAngleOpen = 1800;  // small_servo angle for input 1
const int smallServoAngleResting = 1250;

void setup() {
  // Attach servos to the defined pins
  big_servo.attach(bigServoPin, 970, 2450); // custom range for DS51150-12V servo
  small_servo.attach(smallServoPin, 500, 1550);
  Serial.begin(115200);
  // Set input pins as inputs
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
  pinMode(inputPin4, INPUT);
}

void loop() {
  // Read the input pin states
  int input1State = digitalRead(inputPin1);
  int input2State = digitalRead(inputPin2);
  int input3State = digitalRead(inputPin3);
  int input4State = digitalRead(inputPin4);

  // Determine the angle for big_servo based on input pins 1 and 2
  if (input1State == LOW && input2State == LOW) {
    big_servo.writeMicroseconds(bigServoAngle0);
    Serial.println("Start");
  } else if (input1State == LOW && input2State == HIGH) {
    big_servo.writeMicroseconds(bigServoAngle1);
    Serial.println("CAM");
  } else if (input1State == HIGH && input2State == LOW) {
    big_servo.writeMicroseconds(bigServoAngle2);
    Serial.println("Cut");
  } 

  // Determine the angle for small_servo based on input pin 3
  if (input3State == LOW && input4State == LOW) {
    small_servo.writeMicroseconds(smallServoAngle0);
    Serial.println("Close");
  } else if (input3State == HIGH && input4State == LOW){
    small_servo.writeMicroseconds(smallServoAngle1);
    Serial.println("Open");
  }else if (input4State == HIGH){
    small_servo.writeMicroseconds(smallServoAngleResting);
    Serial.println("Resting");
  }

  // Small delay to debounce input readings
  delay(50);
}
