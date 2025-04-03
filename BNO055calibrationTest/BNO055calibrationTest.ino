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
    Serial.print("Ooops, BNO055(A) not detected");
    while(1);

  }
    adafruit_bno055_offsets_t calibrationData;

    calibrationData.accel_offset_x = -6;
    calibrationData.accel_offset_y = 111;
    calibrationData.accel_offset_z = -10;
  
    calibrationData.gyro_offset_x = 1;
    calibrationData.gyro_offset_y = -2;
    calibrationData.gyro_offset_z = 0;
  
    calibrationData.mag_offset_x = 446;
    calibrationData.mag_offset_y = 15;
    calibrationData.mag_offset_z = 109;
  
    calibrationData.accel_radius = 1000;
    calibrationData.mag_radius = 943;
  
    bno.setSensorOffsets(calibrationData);
  
  
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
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}
