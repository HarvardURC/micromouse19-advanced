/* This minimal example shows how to get single-shot range
measurements from the VL6180X.
The range readings are in units of mm. */

#include <Wire.h>
#include <VL6180X.h>
#include <VL53L0X.h>

VL6180X sensor6;
VL53L0X sensor5;

void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  
  sensor6.init();
  sensor6.configureDefault();
  sensor6.setTimeout(1000);

  sensor5.init();
  sensor5.setTimeout(1000);

}

void loop() 
{ 
  Serial.print("Sensor6 - ");
  Serial.print(sensor6.readRangeSingleMillimeters());
  if (sensor6.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
  
  Serial.print("Sensor5 - ");
  Serial.print(sensor5.readRangeSingleMillimeters());
  if (sensor5.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
