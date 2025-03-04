#include <iostream>
#include <fstream>
#include <vector>
#include <valarray>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>                //https://github.com/adafruit/Adafruit-BMP085-Library
#include <CircularBuffer.hpp>               //https://github.com/rlogiacco/CircularBuffer
#include <time.h>
#include <Forge.h>



Class Forge()
{
  Forge()
  {
    bno = Adafruit_BNO055(55);
    quat = bno.getQuat();
    adxl = Adafruit_ADXL375(375);
    Acceleration = adxl.getVector(Adafruit_ADXL375::VECTOR_ACCELEROMETER);
  }


  float Forge::getChangeInAltitude() //calculates the absolute change in altitude based on the last two readings. note you have to run get altitude 2 times before this is reliable
  {
    return math.abs(OldAltitude - Altitude);
  }


  void Forge::landingVelocityAddData(float point) //adds a new acceleration data point to the circular buffer. used for calculating landing velocity
  {
    accels.push(point);
  }


  void Forge::calculateLandingVelocity() //calculates landing velocity by pulling the current 3 second window, cleaning the values, and integrating over time
  {	
    for(int i=0; i<frequency*window; i++) //removing gravity and negative values then riemann summing to get velocity by multiplying acceleration values by time period of point
    {
      LandingVelocity += (1/frequency)*((accels.pop() - 1));
    }
  }


  void Forge::isMaxAltitude() //gets new altitude and checks it is greater than apogee, if so than replace
  {
    getAltitude();
    if(Altitude > Apogee)
    {	
      Apogee = Altitude;
    }
  }


  void Forge::getAltitude() //moves curent altitude to OldAltitude then gets new value from BMP
  {
    OldAltitude = Altitude;
    Altitude = bmp.readAltitude();
  }


  float Forge::currentAltitude()//returns current altitude
  {
    return Altitude;
  }


  void Forge::getGforce() //gets acceleration from high-g imu and devides by earth's gravity
  {
    float HighAccel = getHighGAcceleration();
    Gforce = getHighGAcceleration() / 9.81f;
    landingVelocityAddData(HighAccel);
  }


  float Forge::getHighGAcceleration() //gets accceleration from high-g imu. helper method for getGforce
  {
    return sqrt(pow(Acceleration.x(),2.0f) + pow(Acceleration.y(),2.0f) + pow(Acceleration.z(),2.0f)));
  }

  void Forge::isMaxGforce() //gets new Gforce and checks it is greater than MaxGForce, if so than replace
  {
    getGforce();
    if(Gforce > MaxGForce)
    {
      MaxGForce = Gforce;
    }
  }


  void Forge::isMaxVelocity() //gets new Velocity and checks it is greater than MaxVelocity, if so than replace
  {
    getVelocity();
    if(Velocity > MaxVelocity )
    {
      MaxVelocity = Velocity;
    }
  }


  void Forge::getVelocity() //gets new Velocity from change in altitude devided by time
  {
    getAltitude();
    Velocity = (getChangeInAltitude()*frequency); //velocity = (change in position (vertical))/(change in time)
  }


  void Forge::recordTime() //sets landing time in format of MM/DD/YYYY|HH:MM:SS.NANO(includesMilisecconds) 
  {
    struct tm local;
    local.tm_sec = myGPS.getSecond();
    local.tm_min = myGPS.getMinute();
    local.tm_hour = myGPS.getHour();
    local.tm_mday = myGPS.getDay();
    local.tm_mon = myGPS.getMonth()-1;//0-11
    local.tm_year = myGPS.getYear()-1900;//years since 1900
    char buffer[80];
    strftime(buffer, 80, "%m/%d/%Y|%H:%M:%S", local);
    LandingTimeString  =  buffer + "." + myGPS.getNanosecond();
    LandingTime = getTime();
  }


  int Forge::getTime()
  {
    return myGPS.getHour()*3600 + myGPS.getMinute()*60 + myGPS.getSecond();
  }


  bool Forge::timer() //determines if 300 secconds (5 min) have passed since landing
  {
    return (300 >= getTime() - LandingTime);
  }


  void Forge::recordTemperature() //records the temperature of the landing site
  {
    Temperature  = bmp.readTemperature(); //temporary placeholder. will use external thermistor on final product
  }


  void Forge::recordOrientation() //gets the orientation from the orientation imu in quaternions. This is orientation of whole capsule, individual stemNAUTS can be derived based on position.
  {
    quat = bno.getQuat();
    Orientation_W = quat.w();
    Orientation_X = quat.x();
    Orientation_Y = quat.y();
    Orientation_Z = quat.z();
  }


  void Forge::recordBatteryStatus() //returns wheather the battery is alive or not (if the battery is dead than this wont run)
  {
    BatteryStatus = true;
  }


  void Forge::calculateSurvivalChance()
  {
    float Lnd_vel = LandingVelocity;
    float MaxG = MaxGForce;
    String Survival = "";
    //Landing velocity should always be under 15 because the parachute 
    //Falls at 3.660648 m/s under main so
    //G forces depend on what humans feel
    //temperature should be okay
    //units in order: m/s^2,m/s,C
    if(MaxG < GThreshold && Lnd_Vel < LandingVelocityThreshold) 
    {
      Survival = "yes";

      if(Temperature < 0 || Temperature > 50)
      {
        //temperatures outside of this range can be survived for  
        //awhile but not for long
        Survival += " but hurry";
      }
    }
    else
    {
      Survival = "Unlikely";
    }

  }


  void Forge::transmitData()
  {
    if(NoRadio)
    {
      Serial.print(toString());
    }
    else
    {
      RadioON;
      delay(2000);
      PttON;
      delay(1000);
      String s = "Temperature of Landing Site: " + Temperature + "C";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Apogee reached: " + Apogee + "m";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Power Status: "  + BatteryStatus + "";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Orientation of On-Board STEMnauts: " + "W: " + Orientation_W + "" + "X: " + Orientation_X + "" + "Y: " + Orientation_Y + "" + "Z: " + Orientation_Z + "";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Time of Landing: " + LandingTime + "";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Maximum Velocity: " + MaxVelocity + "m/s";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Landing Velocity: " + LandingVelocity + "m/s";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "G-Forces Sustained: " + MaxGForce + "G";
      APRS_sendStatus(s.c_str());
      delay(10);
      s = "Calculated STEMnaut Crew Survivability: " + SurvivalChance + "";
      APRS_sendStatus(s.c_str());
      delay(10);      
      PttOFF;
      RadioOFF;
      delay(2000);
    }
  }


  int Forge::getPWMDutyCycle() //determines duty cycle from ELRS PWM reciver. used in determining if shutdown signal has been sent
  {
    int highTime = pulseIn(AnalogPin, HIGH);//Measure the length of the HIGH state
    int lowTime = pulseIn(AnalogPin, LOW); //Measure the duration of the LOW state
    int totalTime = highTime + lowTime;//Calculate the total period of the PWM signal
    if (totalTime == 0)//Avoid division by zero when No signal detected
    {
      return 0;
    }
    int dutyCycle = (highTime * 100) / totalTime;// Calculate the duty cycle as a percentage
    return dutyCycle;
  }
  

  bool Forge::getShutdownStatus() //uses dutycycle to determine whether should shutdown
  {
    return getPWMDutyCycle() >= shutdownThreshold;
  }


  void Forge::shutdown() //shutdown is sumulated with a 10 second pause
  {
    delay(10000);
  }
  

  String Forge::toString() //returns a string with temperature of landing site, Apogee reached, Battery check/power status, Orientation of on-board STEMnauts, Time of landing, Maximum velocity, Landing velocity, G-forces sustained, andCalculated STEMnaut crew survivability
  {
    String str = "Temperature of Landing Site: " + Temperature + "C" + "\n"
    + "Apogee reached: " + Apogee + "m" + "\n"
    + "Power Status: "  + BatteryStatus + "\n"
    + "Orientation of On-Board STEMnauts: " + "W: " + Orientation_W + "|" + "X: " + Orientation_X + "|" + "Y: " + Orientation_Y + "|" + "Z: " + Orientation_Z + "" + "\n"
    + "Time of Landing: " + LandingTime + "\n"
    + "Maximum Velocity: " + MaxVelocity + "m/s" + "\n"
    + "Landing Velocity: " + LandingVelocity + "m/s" + "\n"
    + "G-Forces Sustained: " + MaxGForce + "G" + "\n"
    + "Calculated STEMnaut Crew Survivability: " + SurvivalChance + "%" + "\n";
    return str;
  }
  

}
