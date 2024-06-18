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
const int bigServoAngle0 = 0;   // big_servo angle for input (0,0)
const int bigServoAngle1 = 45;  // big_servo angle for input (0,1)
const int bigServoAngle2 = 90;  // big_servo angle for input (1,0) or (1,1)
const int smallServoAngle0 = 0;   // small_servo angle for input 0
const int smallServoAngle1 = 90;  // small_servo angle for input 1

void setup() {
  // Attach servos to the defined pins
  big_servo.attach(bigServoPin, 550, 2400); // custom range for DS51150-12V servo
  small_servo.attach(smallServoPin);
  Serial.begin(115200);
  // Set input pins as inputs
  pinMode(inputPin1, INPUT);
  pinMode(inputPin2, INPUT);
  pinMode(inputPin3, INPUT);
}

void loop() {
  // Read the input pin states
  int input1State = digitalRead(inputPin1);
  int input2State = digitalRead(inputPin2);
  int input3State = digitalRead(inputPin3);

  // Determine the angle for big_servo based on input pins 1 and 2
  if (input1State == LOW && input2State == LOW) {
    big_servo.write(bigServoAngle0);
    Serial.println("Start");
  } else if (input1State == LOW && input2State == HIGH) {
    big_servo.write(bigServoAngle1);
    Serial.println("CAM");
  } else if (input1State == HIGH && input2State == LOW) {
    big_servo.write(bigServoAngle2);
    Serial.println("Cut");
  } 

  // Determine the angle for small_servo based on input pin 3
  if (input3State == LOW) {
    small_servo.write(smallServoAngle0);
    Serial.println("Close");
  } else {
    small_servo.write(smallServoAngle1);
    Serial.println("Open");
  }

  // Small delay to debounce input readings
  delay(50);
}
