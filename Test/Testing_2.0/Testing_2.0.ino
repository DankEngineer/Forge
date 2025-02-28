#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP085.h>
#include "SparkFun_BNO08x_Arduino_Library.h";

Adafruit_BMP085 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
BNO08x mBNO085;

void setup() 
{
  
  Serial.begin(9600);
  pinMode(A2, INPUT);

  if (!bmp.begin()) 
  {
    SerialUSB.println("Could not find BMP180 sensor!");
    while (1);
  }

 

  if(!adxl.begin()) 
  {
    SerialUSB.print("Ooops, ADXL375 not detected");
    while(1);
  }

  if (!mBNO085.begin(0x4A, Wire, A4, A5)) 
  {
    SerialUSB.print("BNO085 not detected at default I2C address.");
    while(1);
  }
  setReports();
}

void setReports() 
{
  if (mBNO085.enableGeomagneticRotationVector() == true) 
  {
    Serial.println("Geomagnetic Rotation vector enabled");
  } 
  else 
  {
    Serial.println("Failed to enable geomagnetic rotation vector");
  }
}

void loop(void)
{
  SerialUSB.println("-------------------------------------------------------------");
 /* if(1500<pulseIn(A2,HIGH))
   {
     SerialUSB.println("Remote Shutdown Status: Activated");
   }
   else
   {
     SerialUSB.println("Remote Shutdown Status: Inactive");
   }
  */
  //get and print orientation
  if (mBNO085.getSensorEvent() == true) 
  {
    // Check if we got geomagnetic rotation vector data
    if (mBNO085.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) 
    {
      SerialUSB.print("BNO quat|W: ");
      SerialUSB.print(mBNO085.getRoll());

      SerialUSB.print("  BNO quat|X: ");
     SerialUSB.print(mBNO085.getQuatI());

     SerialUSB.print("  BNO quat|Y: ");
     SerialUSB.print(mBNO085.getQuatJ());

     SerialUSB.print("  BNO quat|Z: ");
     SerialUSB.print(mBNO085.getQuatK());

     SerialUSB.println("");
    }
  }
  

  // Get altitude from BMP180
  float altitude = bmp.readAltitude()* 3.28084;
  SerialUSB.print("Altitude: ");
  SerialUSB.print(altitude);
  SerialUSB.println(" feet");

  //get and print acceleration
  sensors_event_t event;
  adxl.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
   SerialUSB.print("ADXL|X: ");
   SerialUSB.print(event.acceleration.x);
   SerialUSB.println(" m/s^2 ");
   SerialUSB.print("ADXL|Y: ");
   SerialUSB.print(event.acceleration.y);
   SerialUSB.println(" m/s^2 ");
   SerialUSB.print("ADXL|Z: ");
   SerialUSB.print(event.acceleration.z);
   SerialUSB.println(" m/s^2 ");

   
  
   //get and print temperature
   int sensorValue = analogRead(A1);
   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
   float voltage = sensorValue * (3.3/1024.0);
   SerialUSB.print((voltage - 0.5)*100);
   SerialUSB.println(" Degrees C");


  delay(100);


}