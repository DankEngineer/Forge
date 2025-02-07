#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>

  
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
void setup() 
{
  Serial.begin(9600);
  if(!bno.begin()) 
  {
    SerialUSB.print("Ooops, BNO055(A) not detected");
    while(1);
  }

  if(!adxl.begin()) 
  {
    SerialUSB.print("Ooops, ADXL375 not detected");
    while(1);
  }

}

void loop(void)
{
  //get and print orientation
  imu::Quaternion quat = bno.getQuat();
  
  SerialUSB.print("  BNO quat|W: ");
  SerialUSB.print(quat.w());

  SerialUSB.print("  BNO quat|X: ");
  SerialUSB.print(quat.x());

  SerialUSB.print("  BNO quat|Y: ");
  SerialUSB.print(quat.y());

  SerialUSB.print("  BNO quat|Z: ");
  SerialUSB.print(quat.z());

  SerialUSB.println("");


  //get and print acceleration
  sensors_event_t event;
  adxl.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
   SerialUSB.print("ADXL|G: ");
   SerialUSB.print(pow(pow(event.acceleration.x,2) + pow(event.acceleration.y,2) + pow(event.acceleration.z,2),0.5));
   SerialUSB.println(" m/s^2 ");
  
  //get and print temperature
  int sensorValue = analogRead(A1);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  float voltage = sensorValue * (3.3/1024.0);
  SerialUSB.print((voltage - 0.5)*100);
  SerialUSB.println(" Degrees C");


  delay(10);


}