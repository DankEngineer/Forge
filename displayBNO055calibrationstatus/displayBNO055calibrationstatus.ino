#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>

  
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if(!bno.begin()) 
  {
    SerialUSB.print("Ooops, BNO055(A) not detected");
    while(1);

  }


}

void loop() {
  // put your main code here, to run repeatedly:
  displayCalStatus();
}


void displayCalStatus(void){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  SerialUSB.print("\t");
  if (!system)
  {
    SerialUSB.print("! ");
  }

  /* Display the individual values */
  SerialUSB.print("Sys:");
  SerialUSB.print(system, DEC);
  SerialUSB.print(" G:");
  SerialUSB.print(gyro, DEC);
  SerialUSB.print(" A:");
  SerialUSB.print(accel, DEC);
  SerialUSB.print(" M:");
  SerialUSB.println(mag, DEC);
}