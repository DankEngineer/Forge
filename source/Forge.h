#ifndef FORGE_H
#define FORGE_H

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

class Forge 
{
  private:
  // Macros
  #define BattPin       A5
  #define GpsPwr        7
  #define PwDwPin       A3
  #define PowerHL       A4
  #define PttPin        3
  #define AnalogPin     A1
  #define PttON         digitalWrite(PttPin, HIGH)
  #define PttOFF        digitalWrite(PttPin, LOW)
  #define RadioON       digitalWrite(PwDwPin, HIGH)
  #define RadioOFF      digitalWrite(PwDwPin, LOW)
  #define RfHiPwr       digitalWrite(PowerHL, HIGH)
  #define RfLowPwr      digitalWrite(PowerHL, LOW)

  // Variables

  //mode
  bool NoRadio = true;

  //reqs from handbook
  float Temperature = 0.0;
  float Apogee = 0.0;
  bool BatteryStatus = true;
  float Orientation_W = 0.0;//-|
  float Orientation_X = 0.0;// |
  float Orientation_Y = 0.0;// | orientation in quaternions
  float Orientation_Z = 0.0;//-|
  String LandingTime = "";
  float MaxVelocity = 0.0;
  float LandingVelocity = 0.0;
  float MaxGForce = 0.0;	
  String SurvivalChance = "";

  //helper variables
  float LandingTimeFloat  = 0.0; //frequency of high-G accelerometer data (Hz)
  float Velocity = 0.0;
  float GForce = 0.0;
  float Altitude = 0.0;
  float OldAltitude = 0.0;
  const int shutdownThreshold = 50; //threshold for shutdown signal (% duty cycle)
  const float GThreshold = 25.0; //survivable GForce threshold
  const float LandingVelocityThreshold = 15.24; //survivable Landing Velocity threshold
  const float frequency = 20.0; //data points per sec for high-g accelerometer
  CircularBuffer<float, frequency * 3> accels;
  Adafruit_BNO055 bno = Adafruit_BNO055(55);
  imu::Quaternion quat = bno.getQuat();
  Adafruit_ADXL375 adxl = Adafruit_ADXL375(375);
  imu::Vector<3> Acceleration = adxl.getVector(Adafruit_ADXL375::VECTOR_ACCELEROMETER);

  //******************************  APRS CONFIG ********************************** //from https://github.com/lightaprs/LightAPRS-2.0/blob/main/LightAPRS-2-vehicle/LightAPRS-2-vehicle.ino
  char    CallSign[7]="NOCALL"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
  int8_t  CallNumber=9;//SSID http://www.aprs.org/aprs11/SSIDs.txt
  char    Symbol='>'; // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
  bool    alternateSymbolTable = false ; //false = '/' , true = '\'

  char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

  char    comment[50] = "Sending payload data for NASA USLI"; //Max 50 char but shorter is better.
  char    StatusMessage[50] = "GO DUKES";
  //*****************************************************************************
   public:
  //Constructor
  Forge();

  //Methods
  float getChangeInAltitude();
  void landingVelocityAddData(float point);
  void calculateLandingVelocity();
  void isMaxAltitude();
  void getAltitude();
  float currentAltitude();
  void getGforce();
  float getHighGAcceleration();
  void isMaxGforce();
  void isMaxVelocity();
  void getVelocity();
  void recordTime();
  bool timer();
  void recordTemperature();
  void recordOrientation();
  void recordBatteryStatus();
  void calculateSurvivalChance();
  void transmitData();
  int getPWMDutyCycle();
  bool getShutdownStatus();
  void shutdown();
  String toString();
};

#endif //FORGE_H
