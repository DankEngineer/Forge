# Forge System README

## Overview
The `Forge` system was developed for the **2025 NASA University Student Launch Initiative (USLI)** competition by the **James Madison University** team.
The `Forge` class provides a system for monitoring and analyzing data from the payload onboard the Vulcan I rocket. This system collects data from sensors, calculates critical metrics such as maximum altitude, velocity, and G-forces, and transmits the results via APRS. It also evaluates the survivability of onboard STEMnauts based on the collected data.

---

## Features
- **Altitude Monitoring**: Tracks and records the maximum altitude (apogee) using a BMP sensor.
- **Velocity Calculation**: Calculates the velocity based on altitude changes.
- **G-Force Measurement**: Measures high-G accelerations and determines maximum G-forces sustained.
- **Temperature Recording**: Records the temperature at the landing site.
- **Orientation Tracking**: Uses quaternion data from a BNO055 absolute orientation IMU to record orientation.
- **Battery Status**: Reports battery status.
- **Survivability Assessment**: Calculates the likelihood of onboard STEMnauts' survival based on velocity, G-forces, and temperature.
- **Data Transmission**: Sends telemetry data via APRS.
- **Shutdown Mechanism**: Includes a shutdown feature triggered by a specific PWM signal.

---

## Sensors Used
- **Ublox MAX-M8Q GPS**: For time and position data.
- **Adafruit BMP085**: For altitude and temperature readings.
- **Dorji DRA818V**: For transmitting data via APRS.
- **LightAPRS 2.0**: A microcontroller integrated with the GPS, altimeter, and radio listed above.
- **Adafruit BNO055**: Absolute orientation IMU for orientation tracking.
- **Adafruit ADXL375**: High-G accelerometer for G-force measurements.
---

## Installation
1. Clone the repository and include the required libraries in your Arduino environment:
   - [Adafruit BMP085 Library](https://github.com/adafruit/Adafruit-BMP085-Library)
   - [Adafruit Sensor Libraries](https://github.com/adafruit/Adafruit_Sensor)
   - [CircularBuffer](https://github.com/rlogiacco/CircularBuffer)
   - [SparkFun Ublox Arduino Library](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library)
   - [ZeroAPRS Library](https://github.com/hakkican/ZeroAPRS)

2. Ensure the sensors are correctly wired to the microcontroller as per their datasheets.

---

## Usage
### Initialization
The `Forge` object is instantiated using its constructor, which initializes the onboard sensors and sets default values for key variables.

### Core Functions
- **Altitude and Velocity**:
  - `getAltitude()`: Reads the current altitude.
  - `isMaxAltitude()`: Updates the maximum altitude if the current altitude exceeds the previous value.
  - `getVelocity()`: Calculates the velocity from changes in altitude.
  - `isMaxVelocity()`: Updates the maximum velocity if the current velocity exceeds the previous value.

- **G-Force Measurement**:
  - `getGforce()`: Updates the current G-force using acceleration data.
  - `isMaxGforce()`: Updates the maximum G-force if the current value exceeds the previous maximum.

- **Landing Conditions and System Status**:
  - `recordTime()`: Records the time of landing.
  - `calculateLandingVelocity()`: Calculates the landing velocity based on 3 seconds of acceleration data recorded at and just before landing.
  - `recordTemperature()`: Records the temperature at the landing site.
  - `recordOrientation()`: Records the orientation of the onboard STEMnauts.
  - `recordBatteryStatus()`: Records the status of the battery.

- **Survivability**:
  - `calculateSurvivalChance()`: Evaluates the survival chances based on landing velocity, G-forces, and temperature.

- **Data Transmission**:
  - `transmitData()`: Sends telemetry data over APRS.

- **Shutdown**:
  - `getShutdownStatus()`: Determines if a shutdown signal has been received.
  - `shutdown()`: Simulates shutdown by pausing operations for 10 seconds (this get looped infinitely).

---

## Data Format
### Telemetry Data sent through APRS
The system generates telemetry data in the following format to be sent as an APRS status messages with a small delay between transmissions:
- Temperature of Landing Site: X°C 
- Apogee reached: Y m
- Power Status: true/false
- Orientation of On-Board STEMnauts: W, X, Y, Z (Quaternions) 
- Time of Landing: MM/DD/YYYY|HH:MM:SS.NANO 
- Maximum Velocity: Z m/s 
- Landing Velocity: A m/s 
- G-Forces Sustained: B G 
- Calculated STEMnaut Crew Survivability: Yes/Yes but hurry/Unlikley
---
## Configuration/User-Defined Settings
### APRS Call Sign
Modify the `CallSign` variable to your CallSign:

    char CallSign[7] = "YOURCALL";
---
### APRS Frequency 
Set the operating frequency based on your use case:

    char Frequency[9] = "144.3900";
---
### Shutdown Threshold 
Adjust the PWM duty cycle threshold for initiating shutdown as needed:

    const int shutdownThreshold = 50;
---
## Notes and Limitations

        1. Ensure all sensors are properly calibrated before deployment.
        2. The system currently uses a rule of thumb for the battery status (if we can transmit, than the battery is working).
