#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);

void setup()
{ 
  Serial.begin(9600);
  pinMode(A2, INPUT);
}

void loop(void)
{

  SerialUSB.println(pulseIn(A2,HIGH));
  if(1500<pulseIn(A2,HIGH))
  {
    SerialUSB.println("Off");
  }
  else
  {
    SerialUSB.println("On : " + pulseIn(A2,HIGH));
  }
}