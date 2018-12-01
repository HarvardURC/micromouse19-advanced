#include <VL6180X.h>
#include <Wire.h>


VL6180X sensor;

int leftWheelB = 23; //A1
int leftWheelA = 22; //A2
int rightWheelA = 20; //B1
int rightWheelB = 21; //B2
int onboardLED = 13; 

int speed;
int SenVal;
int setpoint = 100;

// Initialize PID terms
float k = -0.8;
float Kp = 1.2;
float Ki = 0;
float Kd = 0;
float error;
float last_error = 0;
float integral_error;
float differential_error;
float pTerm;
float dTerm;
float iTerm;
float motor_output;

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

  Wire.begin();

  sensor.init();
  sensor.configureDefault();


  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);

  sensor.setTimeout(500);


  sensor.stopContinuous();

  delay(3000);
  sensor.startInterleavedContinuous(100);
 
}
void setDriveSpeedForward() {
  analogWrite(leftWheelA, speed);
  analogWrite(leftWheelB, 0);

  analogWrite(rightWheelA, speed);
  analogWrite(rightWheelB, 0);
}


void setDriveSpeedBackward() {
  speed = -1*speed;
  
  analogWrite(leftWheelA, 0);
  analogWrite(leftWheelB, speed);

  analogWrite(rightWheelA, 0);
  analogWrite(rightWheelB, speed);
}
/*void setDriveSpeed(int reading) {
  // Going forward 
  if (reading > thres){
    analogWrite(leftWheelA, speed);
    analogWrite(leftWheelB, 0);

    analogWrite(rightWheelA, speed);
    analogWrite(rightWheelB, 0);
  }
  // Going backward
  else if (reading < thres){
    analogWrite(leftWheelA, 0);
    analogWrite(leftWheelB, speed);

    analogWrite(rightWheelA, 0);
    analogWrite(rightWheelB, speed);
  } 
}*/
void loop() {
    SenVal = sensor.readRangeContinuousMillimeters();
    Serial.println(SenVal); 
    Serial.print("             ");
    
    // PID
    error = setpoint - SenVal; //Serial.print(error);
    pTerm = error * Kp;
                
    differential_error = error - last_error;
  
    dTerm = differential_error * Kd;
  
    last_error = error;
                
    integral_error = integral_error + error;
 
    iTerm = integral_error * Ki;
  
    motor_output = k * (pTerm + dTerm + iTerm); 
    Serial.print("             ");
    Serial.print(motor_output);
    Serial.println();
    

    if (motor_output > 255) {
      motor_output = 255;
    }
    if (motor_output < -255){
      motor_output = -255;
    }

    speed = motor_output;
    
    // Drive
    if (motor_output > 0) {
      setDriveSpeedForward();  
    }
    else if (motor_output < 0) {
      setDriveSpeedBackward();
    }
    delay(20);
    
    // Serial Monitor
    

}
