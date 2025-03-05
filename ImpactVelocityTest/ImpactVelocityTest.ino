#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  SerialUSB.println()
  
}
