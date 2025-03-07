#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "SparkFun_BNO08x_Arduino_Library.h";
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>
#include <CircularBuffer.hpp>               //https://github.com/rlogiacco/CircularBuffer


#define BattPin       A5
#define GpsPwr        7
#define PwDwPin       A3
#define PowerHL       A4
#define PttPin        3

//macros
#define GpsON       digitalWrite(GpsPwr, LOW)
#define GpsOFF      digitalWrite(GpsPwr, HIGH)
#define PttON       digitalWrite(PttPin, HIGH)
#define PttOFF      digitalWrite(PttPin, LOW)
#define RadioON     digitalWrite(PwDwPin, HIGH)
#define RadioOFF    digitalWrite(PwDwPin, LOW)
#define RfHiPwr     digitalWrite(PowerHL, HIGH)
#define RfLowPwr    digitalWrite(PowerHL, LOW)
//******************************  APRS CONFIG **********************************
char    CallSign[7]="KR4AIC"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=9;//SSID http://www.aprs.org/aprs11/SSIDs.txt
char    Symbol='>'; // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool    alternateSymbolTable = false ; //false = '/' , true = '\'

char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

char    comment[50] = "LightAPRS 2.0"; // Max 50 char but shorter is better.
char    StatusMessage[80] = "Testing 126";
//*****************************************************************************

//*****************************************************************************

uint16_t  BeaconWait=15;  //seconds sleep for next beacon (HF or VHF). This is optimized value, do not change this if possible.

//******************************  APRS SETTINGS *********************************

//do not change WIDE path settings below if you don't know what you are doing :) 
uint8_t   Wide1=1; // 1 for WIDE1-1 path
uint8_t   Wide2=1; // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.  
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.  
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.  
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles. 
 */
uint8_t pathSize=2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = false; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet)) 
boolean  aliveStatus = true; //for tx status message on first wake-up just once.
boolean radioSetup = false; //do not change this, temp value
//static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1; //increased +1 after every APRS transmission


//******************************  GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed
boolean ublox_high_alt_mode_enabled = false; //do not change this
int16_t GpsInvalidTime=0; //do not change this
boolean gpsSetup=false; //do not change this.
bool satsmode = false;//use gps(satalites) or no
//manual synching (if no satalites)
int Monthmany = 3;
int Daymany = 8;
int Yearmany = 2025;
int Hourmany = 0;// out of 24 ("military time")
int Minmany = 0;
int Secmany = 0;

//********************************************************************************
SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
BNO08x mBNO085;
  enum State {PAD, FLIGHT, LAND, TRANSMIT, SHUTDOWN};
  State currentState = PAD;
  State nextState = FLIGHT;
float OldAltitude = 0.0;
//DO NOT CALL A THE ALTIMETER HERE (calling bmp.readAltitude() screwed stuff up)
float Altitude = 0.0; 
const float frq = 25;
CircularBuffer<float, 50> accels;
CircularBuffer<float, 100> alts;
float LandingVelocity = -999.0;
float Apogee = -690000.0;
float Gforce = 0.0;
float MaxGForce = -999.0;
float Velocity = -999.0;
float MaxVelocity = -999.0;
float Orientation_W = 100.0;
float Orientation_X = 100.0;
float Orientation_Y = 100.0;
float Orientation_Z = 100.0;
float landingTime = 0.0;
float Temperature = -694.20;
int BatteryStatus = 1;
float GThreshold = 25.0;
float LandingVelocityThreshold = 15.24;

long long absTime = 0;
String absTimeStr = "";
int month = 3;
int day = 8;
int year = 2025;
int sats = 0;
bool GpsFirstFix = false;
int stableCounter = 0;
int remoteShutoffCounter = 0;
String Survival = "";
float absalt =0.0;


//set at location
float StartAltitude = 0.0;




//
void setup() 
{
 

  // While the energy rises slowly with the solar panel, 
  // using the analog reference low solves the analog measurement errors.
  analogReference(AR_INTERNAL1V65);
  pinMode(PttPin, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PwDwPin, OUTPUT);
  pinMode(PowerHL, OUTPUT);
  
  GpsOFF;
  PttOFF;
  RadioOFF; 
  RfLowPwr;

  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  //Watchdog.reset();  
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB){;}
  //Watchdog.reset(); 

  SerialUSB.println(F("Starting"));
  Serial1.begin(9600);// for DorjiDRA818V

  APRS_init();
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_setPathSize(2);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setPathSize(pathSize);
  APRS_setGain(2);

  configDra818(Frequency);

  Wire.begin();
  bmp.begin();

  SerialUSB.println(F(""));
  SerialUSB.print(F("APRS (VHF) CallSign: "));
  SerialUSB.print(CallSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(CallNumber);



  
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
      for(int i = 0; i<100; i++)//filling moving avg method
      {
        mooveMe();
      }
      
      
      getAltitude();//priming altitude change
      getAltitude();
      getAltitude();
     reZero();
    if(satsmode)
    {
      initGps(); //initializes gps 
      updateTime();//waits till satalites are found and updates time and date
    }
    else
    {
      manualSync(Monthmany, Daymany, Yearmany,Hourmany, Minmany, Secmany);
    }
  snprintf(StatusMessage, sizeof(StatusMessage), "Begin");
           sendStatus();
  
}

void waitSats()
{
  while(sats > 0)
  {
    SerialUSB.println("No Sats");
  }
}



  void initGps()
  {
    gpsStart();
    SerialUSB.println("gpsStart() success");
    setupUBloxDynamicModel();
    GpsON;
    waitSats();
      if (myGPS.getPVT())
      {
        if ( (myGPS.getFixType() != 0) && (myGPS.getSIV() > 3) ) 
        {
          GpsInvalidTime=0;
          GpsFirstFix = true;
          ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
          SerialUSB.flush();         
        }
      }
    sats = myGPS.getSIV();
  }




  void setupUBloxDynamicModel() 
  {
    // If we are going to change the dynamic platform model, let's do it here.
    // Possible values are:
    // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
    // DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters. 
    if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g)) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
      ublox_high_alt_mode_enabled = true;
    }
  
  } 

  void gpsStart()
  {  
  bool gpsBegin=false;  
  while(!gpsBegin)
  {
    GpsON;
    delay(1000);
    SerialUSB.println("Initialzing Gps...");
    Wire.begin();
    gpsBegin=myGPS.begin();
    if(gpsBegin)break;
    GpsOFF; 
    delay(2000);
  }
   // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  gpsSetup=true;
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
void sleepSeconds(int sec) //sleep in secconds
{
  PttOFF;
  RadioOFF;
  SerialUSB.flush();
  delay(1000*sec);
}


byte configDra818(char *freq)//radio config
{
  RadioON;
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];//"AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000"
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
  SerialUSB.println("RF Config");
  ack[2] = 0;
  while (ack[2] != 0xa)
  {
    if (Serial1.available() > 0) 
    {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial1.read();
    }
  }
  delay(2000);
  RadioOFF;

  if (ack[0] == 0x30) 
  {
      SerialUSB.print(F("Frequency updated: "));
      SerialUSB.print(freq);
      SerialUSB.println(F("MHz"));
  } 
  else 
    {
      SerialUSB.println(F("Frequency update error!!!"));    
    }
  return (ack[0] == 0x30) ? 1 : 0;
}




void sendStatus() //send statusmessage char array
{
  SerialUSB.println(F("Status sending..."));
  RfHiPwr; //DRA Power 1 Watt
  //RfLowPwr; //DRA Power 0.5 Watt
  RadioON;
  delay(2000);
  PttON;
  delay(1000);
  APRS_sendStatus(StatusMessage);
  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);
  SerialUSB.print(F("Status sent - "));
  SerialUSB.println(TxCount);
  TxCount++;
}
  float getChangeInAltitude() //calculates the absolute change in altitude based on the last two readings. note you have to run get altitude 2 times before this is reliable
  {
    return abs(OldAltitude - Altitude);
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

    void isMaxAltitude() //gets new altitude and checks it is greater than apogee, if so than replace
  {
    getAltitude();
    if(Altitude > Apogee)
    {	
      Apogee = Altitude;
    }
  }

    void getAltitude() //moves curent altitude to OldAltitude then gets new value from BMP
  {
    OldAltitude = Altitude;
    Altitude = getAbsAltitude();
  }

  float currentAltitude()//returns current altitude
  {
    return Altitude;
  }

    float getHighGAcceleration() //gets accceleration from high-g imu. helper method for getGforce
  {
    sensors_event_t event;
    adxl.getEvent(&event);
    return sqrt(pow(event.acceleration.x,2.0f) + pow(event.acceleration.y,2.0f) + pow(event.acceleration.z,2.0f));
  }

    void getGforce() //gets acceleration from high-g imu and devides by earth's gravity
  {
    float HighAccel = getHighGAcceleration();
    Gforce = getHighGAcceleration() / 9.81f;
    landingVelocityAddData(HighAccel);
  }

  
  void isMaxGforce() //gets new Gforce and checks it is greater than MaxGForce, if so than replace
  {
    getGforce();
    if(Gforce > MaxGForce)
    {
      MaxGForce = Gforce;
    }
  }

  

  void isMaxVelocity() //gets new Velocity and checks it is greater than MaxVelocity, if so than replace
  {
    getVelocity();
    if(Velocity > MaxVelocity )
    {
      MaxVelocity = Velocity;
    }
  }

    void getVelocity() //gets new Velocity from change in altitude devided by time
    {
    getAltitude();
    Velocity = (getChangeInAltitude()*(frq)); //velocity = (change in position (vertical))/(change in time)
    }

  void recordTime() 
  {
    long long absTim = absTime;
    int hour = absTim / 3600000;
    absTim %= 3600000;
    int min = absTim / 60000;
    absTim %= 60000;
    int second = absTim / 1000;
    int milli = absTim % 1000; 
    
    // Format date and time as MM/DD/YYYY|HH:MM:SS.MS with proper zero-padding
    absTimeStr = (String(month < 10 ? "0" : "") + String(month) + "/" +
                  String(day < 10 ? "0" : "") + String(day) + "/" +
                  String(year) + "|" +
                  (hour < 10 ? "0" : "") + String(hour) + ":" +  // Ensures leading zero
                  (min < 10 ? "0" : "") + String(min) + ":" +
                  (second < 10 ? "0" : "") + String(second) + "." +
                  (milli < 100 ? (milli < 10 ? "00" : "0") : "") + String(milli));
  }

  void updateTime() 
  {
    static unsigned long long millisAtSync = 0;
    static unsigned long long lastAbsTime = 0;

    if (sats > 0) 
    {
        Serial.println("Satellites found: " + String(sats));

        // Sync date from GPS
        month = myGPS.getMonth();
        day = myGPS.getDay();
        year = myGPS.getYear();

        // Capture sync time
        millisAtSync = millis();

        // Compute absolute time in milliseconds
        absTime = (myGPS.getHour() * 3600000ULL) +
                  (myGPS.getMinute() * 60000ULL) +
                  (myGPS.getSecond() * 1000ULL);

        lastAbsTime = absTime;  // Store last valid GPS time
    } 
    else if (satsmode) 
    {
        Serial.println("Waiting for satellites, found: " + String(sats));
        delay(10000);  // Avoid calling updateTime() too often
    } 
    else 
    {
        // Internal clock mode
        if (millisAtSync == 0) {
            millisAtSync = millis();
            absTime = millis();
        } else {
            absTime = lastAbsTime + (millis() - millisAtSync);
        }
    }
  }

  // Function to manually sync time
  void manualSync(int newMonth, int newDay, int newYear, int newHour, int newMin, int newSec) 
  {
    month = newMonth;
    day = newDay;
    year = newYear;

    // Set absolute time based on manual input
    absTime = (newHour * 3600000ULL) +
              (newMin * 60000ULL) +
              (newSec * 1000ULL);

    // Store sync reference
    static unsigned long long millisAtSync = millis();
    static unsigned long long lastAbsTime = absTime;

    Serial.println("Manual time set: " + absTimeStr);
  }
  void reZero()
  {
  delay(5000);
  float tempalt = 0;
     for(int i = 0; i<99; i++) //seting starting alt using avg of 1000 points
      {
        float billy = bmp.readAltitude();
        tempalt += billy;
        SerialUSB.print("ReZero: ");
        SerialUSB.println(billy);
      }
     StartAltitude = tempalt/100;
     SerialUSB.println(StartAltitude);
  }
  float sum = 0;
  float mooveMe() // moving avg over 100 points
  {
    float valuee = bmp.readAltitude(); //m to ft
   sum += valuee;
   alts.push(valuee);
   if(alts.size()>99)
   {
     sum -= alts.first();
     return sum/100;
   }
   return bmp.readAltitude();
  }



  int getTime()
  {
    landingTime = millis(); //myGPS.getHour()*3600 + myGPS.getMinute()*60 + myGPS.getSecond();
  }

    bool timer() //determines if 300 secconds (5 min) have passed since landing
  {
    if(landingTime + millis() > 290)
    {
      return false; //(300 >= getTime() - LandingTime); will use remote shutdown during testing and maybe full scale flight (will have auto for final)
    }
    return true;
  }

  void recordTemperature() //records the temperature of the landing site
  {
    float temptemp = 0.0;
    for(int i = 0; i < 99; i++)
    {
        int sensorValue = analogRead(A1);
        float voltage = sensorValue * (1.6/1024.0);// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
        float temp = (voltage - 0.5)*100;
        temptemp += temp;
    }
    Temperature = temptemp/100;
  }
  
  void recordOrientation() //gets the orientation from the orientation imu in quaternions. This is orientation of whole capsule, individual stemNAUTS can be derived based on position.
  {
  if (mBNO085.getSensorEvent() == true) 
  {
    // Check if we got geomagnetic rotation vector data
    if (mBNO085.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) 
    {    
    	Orientation_W = mBNO085.getQuatReal();
    	Orientation_X = mBNO085.getQuatI();
    	Orientation_Y = mBNO085.getQuatJ();
    	Orientation_Z = mBNO085.getQuatK();
	  }
	}
  }
  void recordBatteryStatus() //returns wheather the battery is alive or not (if the battery is dead than this wont run)
  {
    BatteryStatus = 1;
  }

  void calculateSurvivalChance()
  {
    Survival = "";
    //Landing velocity should always be under 15 because the parachute 
    //Falls at 3.660648 m/s under main so
    //G forces depend on what humans feel
    //temperature should be okay
    //units in order: m/s^2,m/s,C
    if(MaxGForce < GThreshold && LandingVelocity < LandingVelocityThreshold) 
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

  float getTemp()
  {
      int sensorValue = analogRead(A1);
        float voltage = sensorValue * (1.6/1024.0);// Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
        return (voltage- 0.5)*100;
  }


  bool getShutdownStatus() //uses dutycycle to determine whether should shutdown
  {
    if(remoteShutoffCounter > 0)
    {
      remoteShutoffCounter--;
      return false;
    }
    else if(pulseIn(A2,HIGH) == 0)
    {
        remoteShutoffCounter = 1000;
        return false;
    }
    else
    return pulseIn(A2,HIGH) > 1500;
  }


  void shutdown() //shutdown is sumulated with a 100 second pause
  {
    delay(100000);
  }

  void transmitData()
  {
    snprintf(StatusMessage, sizeof(StatusMessage), 
             "A| Temp: %.2f /Ap: %.2f /Bat: %i /Ori: W: %.2f X: %.2f Y: %.2f Z: %.2f", 
             Temperature, Apogee, BatteryStatus, Orientation_W, Orientation_X, Orientation_Y, Orientation_Z);
    sendStatus();
    delay(500);

    snprintf(StatusMessage, sizeof(StatusMessage), 
             "B| Time: %s /MVel: %.2f /LaVel: %.2f /MGfrc: %.2f /Survl: %s", 
             absTimeStr.c_str(), MaxVelocity, LandingVelocity, MaxGForce, Survival.c_str());
    sendStatus();
    delay(1000);

 }

  float getAbsAltitude()
  {
    absalt = mooveMe() - StartAltitude;
    return absalt;
  }






















void loop(void)
{
  switch (currentState) 
  {
	
    case PAD:
    {
      getAltitude();
      SerialUSB.println("PAD|Alt: " + String(Altitude) + " temp: " + getTemp() +" absAlt: " + getAbsAltitude() + " stabilitycount: " + stableCounter);
      
      if(getChangeInAltitude() >= 5 || Altitude > 3 ) //if altitude change is significant enough (not just moving rocket around but an actual liftoff) go to flight stage
      {//failsafe added with threshold of 152.4 m
        stableCounter++;
        if(stableCounter>10)
        {
           currentState = nextState;
           nextState = LAND;
           snprintf(StatusMessage, sizeof(StatusMessage), "State PAD -> FLIGHT| Current Alt: %.2f m", Altitude);
           sendStatus();
           stableCounter = 0;
           recordTemperature();
         }
      }
      else if(getChangeInAltitude() <= 3 && stableCounter > 0 )
         {
          stableCounter--;
         }

      if(getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
      break;
    }

    case FLIGHT:
    {
      isMaxAltitude();
      isMaxGforce();
      isMaxVelocity();
      SerialUSB.println("Flight| Alt: " + String(Altitude) + " G: " + String(Gforce) + " Vel: " + String(Velocity) + " Temp: " + Temperature +"");
      
      if(((getChangeInAltitude() <= 2) && currentAltitude() < -304.8) ||  1 > abs(Altitude) ) //checks for conditions signifying that landing has happened (304.8m = 1000ft)
      {//threshold should be 6.1 
        stableCounter++;
        if(stableCounter>10)
        {
          stableCounter = 0;
          recordTime();
          calculateLandingVelocity();
          currentState = nextState;
          nextState = TRANSMIT;
          snprintf(StatusMessage, sizeof(StatusMessage), "State FLIGHT -> LAND| Current Alt: %.2f m", Altitude);
          sendStatus();
        }
            
      }

      else if(getChangeInAltitude() >= 5 && stableCounter > 0)
        {
         stableCounter--;
        }

      if(getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
      break;
    }

    case LAND:
    {
      recordTemperature();
      recordOrientation();
      recordBatteryStatus();
      calculateSurvivalChance();
      updateTime();
      snprintf(StatusMessage, sizeof(StatusMessage), "State LAND -> TRNASMIT| Current Alt Change: %.2f m", Altitude);
      sendStatus();
      currentState = nextState;
      nextState = SHUTDOWN; //no shutdown check because will never stay in this state
      break;
    }

    case TRANSMIT:
    {
      if(true)
      {
        transmitData();
      }
      else
      {
        currentState = nextState;
        currentState = SHUTDOWN;
      }

      if(getShutdownStatus())
      {
        currentState = SHUTDOWN;				
      }
      break;
    }

    case SHUTDOWN:
    {
      snprintf(StatusMessage, sizeof(StatusMessage), "Shutdown Activated");
      sendStatus();
      shutdown();
      break;
    }

  }
}