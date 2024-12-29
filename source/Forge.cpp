#include <iostream>
#include <fstream>
#include <vector>
#include <valarray>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>;
#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>                //https://github.com/adafruit/Adafruit-BMP085-Library
#include <CircularBuffer.hpp>               //https://github.com/rlogiacco/CircularBuffer


Class Forge()
{
  //macros
  #define PttON       digitalWrite(PttPin, HIGH)
  #define PttOFF      digitalWrite(PttPin, LOW)
  #define RadioON     digitalWrite(PwDwPin, HIGH)
  #define RadioOFF    digitalWrite(PwDwPin, LOW)
  #define RfHiPwr     digitalWrite(PowerHL, HIGH)
  #define RfLowPwr    digitalWrite(PowerHL, LOW)
  
  //mode
  bool NoRadio = true;
  // reqs from handbook
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
  int SurvivalChance = 0;	
  //helper values
  float LandingTimeFloat  = 0.0; //in secconds based on landing time
  float Velocity = 0.0;
  float GForce = 0.0;
  float Altitude = 0.0;
  float OldAltitude = 0.0;
  float frequency = 20; //data points per sec for high-g accelerometer
  CircularBuffer<float, frequency *3> accels;
  Adafruit_BNO055 bno55 = Adafruit_BNO055(55);
  imu::Quaternion quat = bno.getQuat();
  Adafruit_ADXL375 adxl = Adafruit_ADXL375(375);
  imu::Vector<3> Acceleration = adxl.getVector(Adafruit_ADXL375::VECTOR_ACCELEROMETER);

  //******************************  APRS CONFIG ********************************** // from https://github.com/lightaprs/LightAPRS-2.0/blob/main/LightAPRS-2-vehicle/LightAPRS-2-vehicle.ino
  char    CallSign[7]="NOCALL"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
  int8_t  CallNumber=9;//SSID http://www.aprs.org/aprs11/SSIDs.txt
  char    Symbol='>'; // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
  bool    alternateSymbolTable = false ; //false = '/' , true = '\'

  char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

  char    comment[50] = "Sending payload data for NASA USLI"; // Max 50 char but shorter is better.
  char    StatusMessage[50] = "GO DUKES";
  //*****************************************************************************




	Float getChangeInAltitude() //note you have to run get altitude 2 times before this is reliable on main method it calls getaltitude 2 times in setup
	{
	  return math.abs(OldAltitude - Altitude);
	}



	void landingVelocityAddData(float point) //add data to time window that landing velocity will be calculated from window is circular queue so only 3 seconds of data is kept
	{
		accels.push(point);
	}



	Void calculateLandingVelocity() //calculates landing velocity by pulling the current 3 second window, cleaning the values, and integrating over time
	{	
		for(i=0; i>frequency*3; i++) //removing gravity and - values
		{
			accels.unshift(math.abs(accels.pop()) - 1)
		}
		for(i=0; i>frequency*3; i+) //riemann summing to get velocity by multiplying acceleration values by time period of point
		{
      LandingVelocity += (1/frequency)*accels.pop();
    }
	}



  void isMaxAltitude() //gets new altitude and checks it is greater than apogee, if so than replace
  {
    getAltitude();
	  if(Altitude >= Apogee)
    {	
	  	Apogee = Altitude();
    }
  }

  void getAltitude() //moves curent altitude to OldAltitude than gets new value from hardware
  {
    OldAltitude = Altitude
    Altitude = bmp.readAltitude();
  }

  void getGforce() // gets acceleration from high-g imu and devides by earth's gravity
  {
      Gforce = getHighGAcceleration() / 9.81;
  }

  float getHighGAcceleration() // gets accceleration from high-g imu. helper method for getGforce
  {
    return sqrt((Acceleration.x())*(Acceleration.x()) + (Acceleration.y())*(Acceleration.y()) + (Acceleration.z())*(Acceleration.z()));
  }

  void isMaxGforce() //gets new Gforce and checks it is greater than MaxGForce, if so than replace
  {
    getGforce()
	  if(Gforce >= MaxGForce)
    {
		  MaxGForce = Gforce;
    }
  }



  void isMaxVelocity() //gets new Velocity and checks it is greater than MaxVelocity, if so than replace
  {
    getVelocity();
  	if(Velocity >= MaxVelocity )
    {
  		MaxVelocity = Velocity;
    }
  }

  void getVelocity() //gets new Velocity from change in altitude devided by time
  {
    Velocity = (getChangeInAltitude()/(1/frequency));
  }



  void recordTime() // sets landing time in format of MM/DD/YYYY|HH:MM:SS.NANO(includesMilisecconds)
  {
  	LandingTime  = myGPS.getMonth() + "/" + myGPS.getDay() + "/" + myGPS.getYear() + "|" + myGPS.getHour()+ ":" + myGPS.getMinute()+ ":" + myGPS.getSecond() + "." + myGPS.getNanosecond();
    LandingTimeFloat = myGPS.getHour()*60*60 + myGPS.getMinute()*60 + myGPS.getSecond()
  }

  bool timer() // determines if 300 secconds (5 min) have passed since landing
  {
    return (LandingTimeFloat + 300 >= myGPS.getHour()*60*60 + myGPS.getMinute()*60 + myGPS.getSecond());
  }



  void recordTemperature() // records the temperature of the landing site
  {
	  Temperature  = bmp.readTemperature() //temporary placeholder. will use external thermiostor on final product
  }



  void recordOrientation() // gets the orientation from the orientation imu in quaternions. This is orientation of whole capsule, individual stemNAUTS can be derived based on position.
  {
  	Orientation_W = quat.w();
    Orientation_X = quat.x();
    Orientation_Y = quat.y();
    Orientation_Z = quat.z();
  }



  void recordBatteryStatus() // returns wheather the battery is alive or not (if the battery is dead than this wont run)
  {
	  BatteryStatus = true;
  }


	void calculateSurvivalChance()
  {
	  Lnd_vel = calculateLandingVelocity();
	  MaxG = getGforce();
	  String Survival = "";
    //Landing velocity should always be under 15 because the parachute 
    //Falls at 3.660648 m/s under main so
    //G forces depend on what humans feel
    // temperature should be okay
    // units in order: m/s^2,m/s,C
    if(MaxG < 25*9.8 && Lnd_Vel < 15.24 && (0<Temperature<50)) 
    {
	    Survival = "yes";
    }
    //temperatures outside of this range can be survived for  
    //awhile but not for long
    else if(0>Temperature>50)
    {
	    Survival = "yes but hurry";
    }
    else
    {
	    Survival = "Unlikely";
    }

  }


	transmitData()
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
      APRS_sendStatus("Temperature of Landing Site: " + Temperature + "C".c_str());
      delay(10);
      APRS_sendStatus("Apogee reached: " + Apogee + "m".c_str());
      delay(10);
      APRS_sendStatus("Power Status: "  + BatteryStatus + "".c_str());
      delay(10);
      APRS_sendStatus("Orientation of On-Board STEMnauts: " + "W: " + Orientation_W + "" + "X: " + Orientation_X + "" + "Y: " + Orientation_Y + "" + "Z: " + Orientation_Z + "".c_str());
      delay(10);
      APRS_sendStatus("Time of Landing: " + LandingTime + "".c_str());
      delay(10);
      APRS_sendStatus("Maximum Velocity: " + MaxVelocity + "m/s".c_str());
      delay(10);
      APRS_sendStatus("Landing Velocity: " + LandingVelocity + "m/s".c_str());
      delay(10);
      APRS_sendStatus("G-Forces Sustained: " + MaxGForce + "G".c_str());
      delay(10);
      APRS_sendStatus("Calculated STEMnaut Crew Survivability: " + SurvivalChance + "%".c_str());
      delay(10);      
      PttOFF;
      RadioOFF;
    }
  }

	shutdown()
  {
    //hardware shudown here
  }
  
  String toString() // returns a string with temperature of landing site, Apogee reached, Battery check/power status, Orientation of on-board STEMnauts, Time of landing, Maximum velocity, Landing velocity, G-forces sustained, andCalculated STEMnaut crew survivability
  {
    String str = "Temperature of Landing Site: " + Temperature + "C" + "\n"
    + "Apogee reached: " + Apogee + "m" + "\n"
    + "Power Status: "  + BatteryStatus + "\n"
    + "Orientation of On-Board STEMnauts: " + "W: " + Orientation_W + "" + "X: " + Orientation_X + "" + "Y: " + Orientation_Y + "" + "Z: " + Orientation_Z + " + "\n"
    + "Time of Landing: " + LandingTime + "\n"
    + "Maximum Velocity: " + MaxVelocity + "m/s" + "\n"
    + "Landing Velocity: " + LandingVelocity + "m/s" + "\n"
    + "G-Forces Sustained: " + MaxGForce + "G" + "\n"
    + "Calculated STEMnaut Crew Survivability: " + SurvivalChance + "%" + "\n"
    return str;
  }
  

}
