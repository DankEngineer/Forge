#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
//#include "SparkFun_BNO08x_Arduino_Library.h";
#include <Adafruit_BNO055.h>
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>                        //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h>  //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>
#include <CircularBuffer.hpp>  //https://github.com/rlogiacco/CircularBuffer


#define BattPin A5
#define GpsPwr 7
#define PwDwPin A3
#define PowerHL A4
#define PttPin 3

//macros
#define GpsON digitalWrite(GpsPwr, LOW)
#define GpsOFF digitalWrite(GpsPwr, HIGH)
#define PttON digitalWrite(PttPin, HIGH)
#define PttOFF digitalWrite(PttPin, LOW)
#define RadioON digitalWrite(PwDwPin, HIGH)
#define RadioOFF digitalWrite(PwDwPin, LOW)
#define RfHiPwr digitalWrite(PowerHL, HIGH)
#define RfLowPwr digitalWrite(PowerHL, LOW)
//******************************  APRS CONFIG **********************************
char CallSign[7] = "KR4AIC";        //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t CallNumber = 9;              //SSID http://www.aprs.org/aprs11/SSIDs.txt
char Symbol = '>';                  // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool alternateSymbolTable = false;  //false = '/' , true = '\'

char Frequency[9] = "144.3900";  //default frequency. 144.3900 for US, 144.8000 for Europe

char comment[50] = "LightAPRS 2.0";  // Max 50 char but shorter is better.
char StatusMessage[80] = "Testing 126";
//*****************************************************************************

//*****************************************************************************

uint16_t BeaconWait = 15;  //seconds sleep for next beacon (HF or VHF). This is optimized value, do not change this if possible.

//******************************  APRS SETTINGS *********************************

//do not change WIDE path settings below if you don't know what you are doing :)
uint8_t Wide1 = 1;  // 1 for WIDE1-1 path
uint8_t Wide2 = 1;  // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.  
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.  
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.  
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles. 
 */
uint8_t pathSize = 2;                 // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = false;  //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet))
boolean aliveStatus = true;           //for tx status message on first wake-up just once.
boolean radioSetup = false;           //do not change this, temp value
//static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1;  //increased +1 after every APRS transmission


//******************************  GPS SETTINGS   *********************************
int16_t GpsResetTime = 1800;                  // timeout for reset if GPS is not fixed
boolean ublox_high_alt_mode_enabled = false;  //do not change this
int16_t GpsInvalidTime = 0;                   //do not change this
boolean gpsSetup = false;                     //do not change this.
bool satsmode = false;                        //use gps(satalites) or no
//manual synching (if no satalites)
int Monthmany = 3;
int Daymany = 8;
int Yearmany = 2025;
int Hourmany = 11;  // out of 24 ("military time")
int Minmany = 29;
int Secmany = 15;

//********************************************************************************
SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO055_ADDRESS_A);
//BNO08x mBNO085;

enum State { PAD,
             FLIGHT,
             LAND,
             TRANSMIT,
             SHUTDOWN };
State currentState = PAD;
State nextState = FLIGHT;
float OldAltitude = 0.0;
//DO NOT CALL A THE ALTIMETER HERE (calling bmp.readAltitude() screwed stuff up)
float Altitude = 0.0;
const float frq = 14;  //data rate during flight state
CircularBuffer<float, 30> accels;
CircularBuffer<float, 100> timez;
//CircularBuffer<float, 100> alts;
CircularBuffer<float, 3> vels;
CircularBuffer<float, 30> vels2;
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
long long landingTime = 0;
float Temperature = -694.20;
int BatteryStatus = 1;
float GThreshold = 25.0;
float LandingVelocityThreshold = 15.24;
float valuee = 0.0;
long long absTime = 0;  // now stores flight duration in miliseconds
String absTimeStr = "";
int month = 3;
int day = 8;
int year = 2025;
int sats = 0;
bool GpsFirstFix = false;
int stableCounter = 0;
int remoteShutoffCounter = 0;
String Survival = "";
float absalt = 0.0;
static unsigned long long millisAtSync = 0;

static unsigned long long lastAbsTime = 0;


//set at location
float StartAltitude = 0.0;

//times
unsigned long long launchTime = 0;
unsigned long long landTime = 0;
bool failsafed1 = false;
bool failsafed2 = false;
unsigned long long aoldTime = 0;
unsigned long long anewTime = 1;// is 1 to prevent divide by 0



void setup() 
{
  Wire.end();
  Wire.begin();
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
  RfHiPwr;

  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  //Watchdog.reset();
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB) { ; }
  //Watchdog.reset();

  SerialUSB.println(F("Starting"));
  Serial1.begin(9600);  // for DorjiDRA818V

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


  bmp.begin();

  SerialUSB.println(F(""));
  SerialUSB.print(F("APRS (VHF) CallSign: "));
  SerialUSB.print(CallSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(CallNumber);




  Serial.begin(9600);
  pinMode(A2, INPUT);

  if (!bmp.begin()) {
    SerialUSB.println("Could not find BMP180 sensor!");
    while (1)
      ;
  }



  if (!adxl.begin()) {
    SerialUSB.print("Ooops, ADXL375 not detected");
    while (1)
      ;
  }

  //  if (!mBNO085.begin(0x4A, Wire, A4, A5))
  //  {
  //    SerialUSB.print("BNO085 not detected at default I2C address.");
  //    while(1);
  //  }

  if (!bno.begin()) {
    SerialUSB.print("Ooops, BNO055(A) not detected");
    while (1)
      ;
  }

  //  mBNO085.softReset();

  for (int i = 0; i < 10; i++)  //filling moving avg method
  {
    getVelocity();
  }

  delay(5000);    //wait for altimiter to warm up or something
  getAltitude();  //priming altitude change
  getAltitude();
  getAltitude();
  reZero();
  //setReports();
  if (satsmode) {
    initGps();     //initializes gps
    updateTime();  //waits till satalites are found and updates time and date
  } else {
    manualSync();
  }
  startSign();
  //setReports();
  //calibrate BNO055
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
  displayCalStatus();
  bno.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)0x06);
  bno.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)0x00);
  bno.setExtCrystalUse(true);



}
void startSign()
{
    char Frequency2[9] = "145.0900";
    configDra818(Frequency2);
    snprintf(StatusMessage, sizeof(StatusMessage), "Begin");
    sendStatus();
    configDra818(Frequency);
}

void waitSats() {
  while (sats > 0) {
    SerialUSB.println("No Sats");
  }
}
void resetI2C() {
  Wire.end();    // End the current I2C session
  delay(100);    // Wait for a bit
  Wire.begin();  // Reinitialize the I2C bus
}

void displayCalStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  SerialUSB.print("\t");
  if (!system) {
    SerialUSB.print("! ");
  }

  /* Display the individual values */
  SerialUSB.print("Sys:");
  SerialUSB.print(system, DEC);
  SerialUSB.print(" G:");
  SerialUSB.print(gyro, DEC);
  SerialUSB.print(" A:");
  SerialUSB.print(accel, DEC);
  SerialUSB.print(" M:");
  SerialUSB.println(mag, DEC);
}




void stupidtimefix() {
  month = Monthmany;
  day = Daymany;
  year = Yearmany;

  // Set absolute time based on manual input
  //absTime = (Hourmany * 3600000) +
  //        (Minmany * 60000) +
  //      (Secmany * 1000) + millis();
  absTime = landTime - launchTime;
  landingTime = millis();
}


void initGps() {
  gpsStart();
  SerialUSB.println("gpsStart() success");
  setupUBloxDynamicModel();
  GpsON;
  waitSats();
  if (myGPS.getPVT()) {
    if ((myGPS.getFixType() != 0) && (myGPS.getSIV() > 3)) {
      GpsInvalidTime = 0;
      GpsFirstFix = true;
      ublox_high_alt_mode_enabled = false;  //gps sleep mode resets high altitude mode.
      SerialUSB.flush();
    }
  }
  sats = myGPS.getSIV();
}




void setupUBloxDynamicModel() {
  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE
  // DYN_MODEL_AIRBORNE4g model increases ublox max. altitude limit from 12.000 meters to 50.000 meters.
  if (myGPS.setDynamicModel(DYN_MODEL_AIRBORNE4g))  // Set the dynamic model to DYN_MODEL_AIRBORNE4g
  {
    ublox_high_alt_mode_enabled = true;
  }
}

void gpsStart() {
  bool gpsBegin = false;
  while (!gpsBegin) {
    GpsON;
    delay(1000);
    SerialUSB.println("Initialzing Gps...");
    gpsBegin = myGPS.begin();
    if (gpsBegin) break;
    GpsOFF;
    delay(2000);
  }
  // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0);           //Disable the UART1 port output
  myGPS.setUART2Output(0);           //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();         //Save the current settings to flash and BBR
  gpsSetup = true;
}

//void setReports()
//{
//  if (mBNO085.enableGeomagneticRotationVector() == true)
//  {
//    SerialUSB.println("Geomagnetic Rotation vector enabled");
//  }
//  else
//  {
//    SerialUSB.println("Failed to enable geomagnetic rotation vector");
//  }
//}

void sleepSeconds(int sec)  //sleep in secconds
{
  PttOFF;
  RadioOFF;
  SerialUSB.flush();
  delay(1000 * sec);
}


byte configDra818(char *freq)  //radio config
{
  RadioON;
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];  //"AT+DMOSETGROUP=0,144.8000,144.8000,0000,4,0000"
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
  SerialUSB.println("RF Config");
  ack[2] = 0;
  while (ack[2] != 0xa) {
    if (Serial1.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial1.read();
    }
  }
  delay(2000);
  RadioOFF;

  if (ack[0] == 0x30) {
    SerialUSB.print(F("Frequency updated: "));
    SerialUSB.print(freq);
    SerialUSB.println(F("MHz"));
  } else {
    SerialUSB.println(F("Frequency update error!!!"));
  }
  return (ack[0] == 0x30) ? 1 : 0;
}




void sendStatus()  //send statusmessage char array
{
  SerialUSB.println(F("Status sending..."));
  RfHiPwr;  //DRA Power 1 Watt
  //RfLowPwr; //DRA Power 0.5 Watt
  RadioON;
  delay(2000);
  PttON;
  delay(1000);
  APRS_sendStatus(StatusMessage);
  delay(1000);
  PttOFF;
  RadioOFF;
  delay(100);
  SerialUSB.print(F("Status sent - "));
  SerialUSB.println(TxCount);
  TxCount++;
}
float getChangeInAltitude()  //calculates the absolute change in altitude based on the last two readings. note you have to run get altitude 2 times before this is reliable
{
  return abs(OldAltitude - Altitude);
}

float getChangeInTime()  //calculates the absolute change in altitude based on the last two readings. note you have to run get altitude 2 times before this is reliable
{
  return (anewTime - aoldTime)/1000.0;
}

void landingVelocityAddData(float point)  //adds a new acceleration data point to the circular buffer. used for calculating landing velocity
{
  accels.unshift(point);
}


void calculateLandingVelocity()  //calcs landingvelo from getting 100points of data before landing and avging the oldest 20 (should be velocity before landing)
{
  float lvl = 0.0;
  for (int i = 0; i < 20; i++)  
  {
    lvl += accels.pop();
  }
  LandingVelocity = (lvl/20)-5;
}

void isMaxAltitude()  //gets new altitude and checks it is greater than apogee, if so than replace
{
  getAltitude();
  if (Altitude > Apogee) {
    Apogee = Altitude;
  }
}

void getAltitude()  //moves curent altitude to OldAltitude then gets new value from BMP
{
  OldAltitude = Altitude;
  Altitude = getAbsAltitude();
  aoldTime = anewTime;
  anewTime = millis();
}

float currentAltitude()  //returns current altitude
{
  return Altitude;
}

float getHighGAcceleration()  //gets accceleration from high-g imu. helper method for getGforce
{
  sensors_event_t event;
  adxl.getEvent(&event);
  return sqrt(pow(event.acceleration.x, 2.0f) + pow(event.acceleration.y, 2.0f) + pow(event.acceleration.z, 2.0f));
}

void getGforce()  //gets acceleration from high-g imu and devides by earth's gravity
{
  float HighAccel = getHighGAcceleration();
  Gforce = HighAccel / 9.81f;         // used in max gforce
  
}


void isMaxGforce()  //gets new Gforce and checks it is greater than MaxGForce, if so than replace
{
  getGforce();
  if (Gforce > MaxGForce) {
    MaxGForce = Gforce;
  }
}



void isMaxVelocity()  //gets new Velocity and checks it is greater than MaxVelocity, if so than replace
{
  getVelocity();
  if (Velocity > MaxVelocity) {
    MaxVelocity = Velocity;
  }
  if(Altitude<76.2)
  {
    landingVelocityAddData(Velocity);  // velocity to buffer
  }

}

void getVelocity()  //gets new Velocity from change in altitude devided by time
{
  getAltitude();
  Velocity = mooveMe();  //velocity = moving avg of 10 points calced by (change in position (vertical))/(change in time)
}

void recordTime() {
  long long absTim = absTime;
  int hour = absTim / 3600000;
  absTim %= 3600000;
  int min = absTim / 60000;
  absTim %= 60000;
  int second = absTim / 1000;
  int milli = absTim % 1000;

  // Format time as HH:MM:SS.MS with proper zero-padding
  absTimeStr = ((hour < 10 ? "0" : "") + String(hour) + ":" +  // Ensures leading zero
                (min < 10 ? "0" : "") + String(min) + ":" + (second < 10 ? "0" : "") + String(second) + "." + (milli < 100 ? (milli < 10 ? "00" : "0") : "") + String(milli));
}

void updateTime() {
  millisAtSync = 0;
  lastAbsTime = 0;

  if (sats > 0) {
    Serial.println("Satellites found: " + String(sats));

    // Sync date from GPS
    month = myGPS.getMonth();
    day = myGPS.getDay();
    year = myGPS.getYear();

    // Capture sync time
    millisAtSync = millis();

    // Compute absolute time in milliseconds
    absTime = (myGPS.getHour() * 3600000) + (myGPS.getMinute() * 60000) + (myGPS.getSecond() * 1000);

    lastAbsTime = absTime;  // Store last valid GPS time
  } else if (satsmode) {
    Serial.println("Waiting for satellites, found: " + String(sats));
    delay(10000);  // Avoid calling updateTime() too often
  } else {
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
void manualSync() {
  month = Monthmany;
  day = Daymany;
  year = Yearmany;

  // Set absolute time based on manual input
  absTime = (Hourmany * 3600000) + (Minmany * 60000) + (Secmany * 1000);

  // Store sync reference
  millisAtSync = millis();
  lastAbsTime = absTime;

  Serial.println("Manual time set: " + absTimeStr);
}

void reZero() {
  float tempalt = 0;
  for (int i = 0; i < 30; i++)  //seting starting alt using avg of 100 points
  {
    SerialUSB.println("ReZeroing");
    mooveMe2();
  } 
  SerialUSB.println(StartAltitude);
}


float sum = 0;
float mooveMe()  // moving avg over 3 points
{
  valuee = (getChangeInAltitude() / getChangeInTime());
  sum += valuee;
  vels.push(valuee);
  if (vels.size() > 2) 
  {
    sum -= vels.first();
    return sum / 3;
  }
  return -1;
}

float sum2 = 0;
float mooveMe2()  // moving avg over 3 points
{
  float valu = bmp.readAltitude();
  sum2 += valu;
  vels2.push(valu);
  if (vels2.size() > 29) 
  {
    
    StartAltitude = sum2 / 30;
    sum2 -= vels2.first();
    return sum2 / 30;
  }
  return -1;
}



int getTime() 
{
  landingTime = millis();  //myGPS.getHour()*3600 + myGPS.getMinute()*60 + myGPS.getSecond();
}

bool timer()  //determines if 300 secconds (5 min) have passed since landing
{
  if (millis() - landingTime > 290000) //290 sec is less than 5 mins
  {
    return false;  //(300 >= getTime() - LandingTime); will use remote shutdown during testing and maybe full scale flight (will have auto for final)
  }
  SerialUSB.println((millis() - landingTime) / 1000);
  return true;
}

void recordTemperature()  //records the temperature of the landing site
{
  float temptemp = 0.0;
  for (int i = 0; i < 99; i++) {
    int sensorValue = analogRead(A1);
    float voltage = sensorValue * (1.6 / 1024.0);  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
    float temp = (voltage - 0.5) * 100;
    temptemp += temp;
  }
  Temperature = temptemp / 100;
}

void recordOrientation()  //gets the orientation from the orientation imu in quaternions. This is orientation of whole capsule, individual stemNAUTS can be derived based on position.
{
  imu::Quaternion quat = bno.getQuat();
  //if (mBNO085.getSensorEvent() == true)
  //{
  //SerialUSB.println("bno got sensor event");
  // Check if we got geomagnetic rotation vector data
  //if (mBNO085.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR)
  //{
  //Orientation_W = mBNO085.getQuatReal();
  Orientation_W = quat.w();
  //Orientation_X = mBNO085.getQuatI();
  Orientation_X = quat.x();
  //Orientation_Y = mBNO085.getQuatJ();
  Orientation_Y = quat.y();
  //Orientation_Z = mBNO085.getQuatK();
  Orientation_Z = quat.z();
  //SerialUSB.println("numbers changed");
  //}
}

void recordBatteryStatus()  //returns wheather the battery is alive or not (if the battery is dead than this wont run)
{
  BatteryStatus = 1;
}

void calculateSurvivalChance() {
  Survival = "";
  //Landing velocity should always be under 15 because the parachute
  //Falls at 3.660648 m/s under main so
  //G forces depend on what humans feel
  //temperature should be okay
  //units in order: m/s^2,m/s,C
  if (MaxGForce < GThreshold && LandingVelocity < LandingVelocityThreshold) {
    Survival = "Y";

    if (Temperature < 0 || Temperature > 50) {
      //temperatures outside of this range can be survived for
      //awhile but not for long
      Survival += "/C";
    }
  } else {
    Survival = "N";
  }
}

float getTemp() {
  int sensorValue = analogRead(A1);
  float voltage = sensorValue * (1.6 / 1024.0);  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  return (voltage - 0.5) * 100;
}


bool getShutdownStatus()  //uses dutycycle to determine whether should shutdown
{
  if (remoteShutoffCounter > 0) {
    remoteShutoffCounter--;
    return false;
  } else if (pulseIn(A2, HIGH) == 0) {
    remoteShutoffCounter = 1000;
    return false;
  } else
    return pulseIn(A2, HIGH) > 1500;
}


void shutdown()  //shutdown is sumulated with a 1000 second pause
{
  delay(1000000);//1000 sec = 1000000 milisec
}

void transmitData() {
  snprintf(StatusMessage, sizeof(StatusMessage),
           "A| Temp: %.2f /Ap: %.2f /Bat: %i /Ori: W: %.2f X: %.2f Y: %.2f Z: %.2f",
           Temperature, Apogee, BatteryStatus, Orientation_W, Orientation_X, Orientation_Y, Orientation_Z);
  sendStatus();
  delay(500);

  SerialUSB.println(Survival);

  snprintf(StatusMessage, sizeof(StatusMessage),
           "B| Time: %s /MxVl: %.2f /LaVl: %.2f /MxG: %.2f /Svl: %s",
           absTimeStr.c_str(), MaxVelocity, LandingVelocity, MaxGForce, Survival.c_str());
  sendStatus();
  delay(1000);
}

float getAbsAltitude() {
  valuee = bmp.readAltitude();
  absalt = valuee - StartAltitude;
  return absalt;
}

bool failsafe1() {
  if (Altitude > 50) {
    failsafed1 = true;
    return true;
  }
  return false;
}

bool failsafe2() {
  if (millis() < (launchTime + 200000))//remember number is in MILISECONDS (200000 = 200 sec)
  {
    failsafed2 = true;
    return true;
  }
  return false;
}










unsigned long long tensectmr = 0.0;













int thingg = 1;

void loop(void)
{
    isMaxAltitude();
    isMaxGforce();
    isMaxVelocity();
    SerialUSB.println("Test| Alt:" + String(Altitude) +  " Vel:" + String(Velocity) + " MaxVelo:" + String(MaxVelocity) + " LandVelo:" + LandingVelocity +  " LVbuffrsiz:" + String(accels.size()) + "");
     //if(accels.size() == 100)
     //calculateLandingVelocity();
     
     if (getShutdownStatus() && thingg == 1) 
     {
         calculateLandingVelocity();
         thingg--;
     }
      
}