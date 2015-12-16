/* 
 * Author: Sandor Laki, lakis@inf.elte.hu
 *
 * Description:
 * Connecting an IMU MPU-9150 sensor to Arduino
 *
 * Created on 2015


This code requires the installation of RTIMULib-Arduino. You should copy the content of libraries to <your documents>/Arduino/libraries
The library is available on: https://github.com/richards-tech/RTIMULib-Arduino

Settings for Drotek IMU MPU-9150:
 MPU-side ---------------- Arduino Uno
     SCL ----------------- A5
     SDA ----------------- A4
     VDD ----------------- 3.3V
     GND ----------------- GND 
     INT -X do not connect!!!
 
 Comment and uncomment the following lines in libraries/RTIMULib/RTIMULibDefs.h
 //  IMU enable defs - only one should be enabled, the rest commented out
 //#define MPU9150_68                      // MPU9150 at address 0x68
 #define MPU9150_69                      // MPU9150 at address 0x69  <--------------------------------------
 //#define MPU9250_68                      // MPU9250 at address 0x68
 //#define MPU9250_69                      // MPU9250 at address 0x69
 ...
 
 Only MPU9150_69 should be uncommented since Drotek MPU uses 0x69 address for I2C communication
 
 */
 
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include <EEPROM.h>

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  300                         // interval between pose displays

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  9600

unsigned long lastDisplay;
unsigned long lastRate;
int sampleCount;

void setup()
{
    int errcode;
  
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin();
    imu = RTIMU::createIMU(&settings);                        // create the imu object
  
    Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

    lastDisplay = lastRate = millis();
    sampleCount = 0;

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.
    
    fusion.setSlerpPower(0.02);
    
    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor
    
    fusion.setGyroEnable(true);
    fusion.setAccelEnable(true);
    fusion.setCompassEnable(true);
}

void loop()
{  
    unsigned long now = millis();
    unsigned long delta;
    int loopCount = 1;
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;
        fusion.newIMUData(imu->getGyro(), imu->getAccel(), imu->getCompass(), imu->getTimestamp());
        sampleCount++;
        if ((delta = now - lastRate) >= 1000) {
            Serial.print("Sample rate: "); Serial.print(sampleCount);
            if (imu->IMUGyroBiasValid())
                Serial.println(", gyro bias valid");
            else
                Serial.println(", calculating gyro bias");
        
            sampleCount = 0;
            lastRate = now;
        }
        if ((now - lastDisplay) >= DISPLAY_INTERVAL) {
            lastDisplay = now;
            //RTMath::display("Gyro:", (RTVector3&)imu->getGyro());                // gyro data
            RTMath::display("Accel:", (RTVector3&)imu->getAccel());              // accel data
            //RTMath::display("Mag:", (RTVector3&)imu->getCompass());              // compass data
            //RTMath::displayRollPitchYaw("Pose:", (RTVector3&)fusion.getFusionPose()); // fused output
            Serial.println();
        }
    }
}

