#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP085.h>
#include <CircularBuffer.hpp> 

Adafruit_BMP085 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
float LandingVelocity = 0;
const float frq = 1;
float StartAltitude = 0.0;
bool appReached = false;
float point = 0.0;

CircularBuffer<float, 20> accels;

  float getAbsAltitude(){

    return (bmp.readAltitude()- StartAltitude);

  }

  void landingVelocityAddData(float point) //adds a new acceleration data point to the circular buffer. used for calculating landing velocity
  {
    accels.push(point);
  }

  void calculateLandingVelocity() //calculates landing velocity by pulling the current 3 second window, cleaning the values, and integrating over time
  {	
    float lvl = 0.0;
    for(int i=0; i<accels.size(); i++) //riemann summing to get velocity by multiplying acceleration values by time period of point
    {
      
      lvl += (1/frq)*((accels.pop() - 1)); // accels is in erms of g force and -1 removes gravity which is than multiplied by 1/the frequency (time between points) to get velocity
    }
    LandingVelocity = lvl;
  }

  float getHighGAcceleration() //gets accceleration from high-g imu. helper method for getGforce
  {
    sensors_event_t event;
    adxl.getEvent(&event);
    return sqrt(pow(event.acceleration.x,2.0f) + pow(event.acceleration.y,2.0f) + pow(event.acceleration.z,2.0f));
  }


void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  accels.push(point);
  
  if(getAbsAltitude()>10){
    bool appReached = true;
  }

  if((getAbsAltitude()<5) && appReached && (getHighGAcceleration()>20)){
    calculateLandingVelocity();
  } 

  SerialUSB.print("Landing Vel: " + String(LandingVelocity) + "Gs: " + getHighGAcceleration()+" absAlt: " + getAbsAltitude());

}
