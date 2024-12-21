#ifndef FORGE_H
#define FORGE_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BNO055.h>
#include <CircularBuffer.h>
#include <imu.h>

class Forge {
public:
    // Macros
    #define PttON       digitalWrite(PttPin, HIGH)
    #define PttOFF      digitalWrite(PttPin, LOW)
    #define RadioON     digitalWrite(PwDwPin, HIGH)
    #define RadioOFF    digitalWrite(PwDwPin, LOW)
    #define RfHiPwr     digitalWrite(PowerHL, HIGH)
    #define RfLowPwr    digitalWrite(PowerHL, LOW)

    // Variables
    bool NoRadio = true;
    float Temperature = 0.0;
    float Apogee = 0.0;
    bool BatteryStatus = true;
    float Orientation = 0.0;
    String LandingTime = "";
    float MaxVelocity = 0.0;
    float LandingVelocity = 0.0;
    float MaxGForce = 0.0;
    int SurvivalChance = 0;
    float LandingTimeFloat = 0.0;
    float Velocity = 0.0;
    float GForce = 0.0;
    float Altitude = 0.0;
    float OldAltitude = 0.0;
    float frequency = 20.0; // Data points per second
    CircularBuffer<float, 60> accels;

    Adafruit_BNO055 bno;
    HighGIMU HighGIMU;
    imu::Vector<3> Acceleration;

    // APRS Configuration
    char CallSign[7] = "NOCALL";
    int8_t CallNumber = 9;
    char Symbol = '>';
    bool alternateSymbolTable = false;
    char Frequency[9] = "144.3900";
    char comment[50] = "Sending payload data for NASA USLI";
    char StatusMessage[50] = "GO DUKES";

    // Constructor
    Forge();

    // Methods
    float getChangeInAltitude();
    void landingVelocityAddData(float point);
    void calculateLandingVelocity();
    void isMaxAltitude();
    void getAltitude();
    void getGForce();
    float getHighGAcceleration();
    void isMaxGForce();
    void isMaxVelocity();
    void getVelocity();
    void recordTime();
    bool timer();
    void recordTemperature();
    void recordOrientation();
    void recordBatteryStatus();
    void calculateSurvivalChance();
    void transmitData();
    void shutdown();
    String toString();

private:
    void APRS_sendStatus(const char* message);
};

#endif // FORGE_H