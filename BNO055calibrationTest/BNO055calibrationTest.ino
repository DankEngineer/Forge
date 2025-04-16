#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
//#include "SparkFun_BNO08x_Arduino_Library.h";
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
//#include <avr/dtostrf.h>
#include <MemoryFree.h>
//#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>
#include <CircularBuffer.hpp>               //https://github.com/rlogiacco/CircularBuffer
  
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
      float check = 0;
      float Sw = 1;
      float Sx = 0;
      float Sy = 0;
      float Sz = 0;


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
  
    bno.setMode(OPERATION_MODE_IMUPLUS);

  bno.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x06);
  bno.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x00);
  bno.setExtCrystalUse(true);







}

void loop() {
  // put your main code here, to run repeatedly:
      imu::Quaternion quat = bno.getQuat();


    if (check == 0){
      
      Sw= quat.w();
      Sx = quat.x();
      Sy = quat.y();
      Sz = quat.z();
      Serial.print("zeroed");
      check = 1;
    }
  displayCalStatus();
  recordOrientation();
}

void recordOrientation() //gets the orientation from the orientation imu in quaternions. This is orientation of whole capsule, individual stemNAUTS can be derived based on position.
  {
    imu::Quaternion quat = bno.getQuat();
 

      


  //if (mBNO085.getSensorEvent() == true) 
  //{
    //Serial.println("bno got sensor event");
    // Check if we got geomagnetic rotation vector data
    //if (mBNO085.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) 
    //{    
    	//Orientation_W = mBNO085.getQuatReal();
      Serial.print("W: ");
      Serial.print(quat.w());
    	//Orientation_X = mBNO085.getQuatI();
      Serial.print("X: ");
      Serial.print(quat.x());
    	//Orientation_Y = mBNO085.getQuatJ();
      Serial.print("Y: ");
      Serial.print(quat.y());
    	//Orientation_Z = mBNO085.getQuatK();
      Serial.print("Z: ");
      Serial.print(quat.z());
      //Serial.println("numbers changed");
      
	  //}
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
