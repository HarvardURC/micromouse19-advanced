#include <VL6180X.h>

VL6180X sensor;

int leftWheelB = 23; //A1
int leftWheelA = 22; //A2
int rightWheelA = 20; //B1
int rightWheelB = 21; //B2
int onboardLED = 13; 

int speed = 50;

void setup() {
  // Pin configuration
  pinMode(leftWheelB, OUTPUT);
  pinMode(leftWheelA, OUTPUT);
  pinMode(rightWheelB, OUTPUT);
  pinMode(rightWheelA, OUTPUT);
  
  pinMode(onboardLED, OUTPUT);
  
  // Show when robot is on
  digitalWrite(onboardLED, HIGH);
  
  Serial.begin(9600);
}


void loop() {

    
    analogWrite(leftWheelA, speed);
    analogWrite(leftWheelB, 0);

    analogWrite(rightWheelA, speed);
    analogWrite(rightWheelB, 0);

    delay(1000);

    analogWrite(leftWheelA, 0);
    analogWrite(leftWheelB, speed);
    
    analogWrite(rightWheelA, 0);
    analogWrite(rightWheelB, speed);

    delay(1000);
}
