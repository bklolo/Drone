#include <Servo.h> // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <ArduPID.h>
#include <SD.h>
#include <SPI.h>
#include "Drone_Funcs.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU instance
bool isCalibrated;

void setup()
{
    Serial.begin(9600); // default baudrate

    initESCs();
    testSD();
    delay(500);
    initBNO055(&bno);
    initPIDs();
}

void loop()
{
    // Get a new BNO sensor event
    sensors_event_t event;
    bno.getEvent(&event);
    Quad error, pos;

    if (isCalibrated)
    {
        calculateResponse(event, &error, &pos);
        writeToESCsRoll(error.roll, pos.roll);
        writeToESCsPitch(error.pitch, pos.pitch);
        writeToESCsYaw(error.yaw, pos.yaw);
    }
    else
    {
        calibrate(&bno, event);
        if (Serial.available() > 0)
        {
            if (Serial.read() == '0')
            {
                normalizeInput(event);
                isCalibrated = true;
                Serial.println("Outputs have been normalized!");
            }
        }
    }

    //
    if (Serial.available() > 0)
    {
        if (Serial.read() == '0')
        {
            normalizeInput(event);
            Serial.println("Outputs have been normalized!");
        }
    }

    //readContData();   // get serial data from HOTAS controller

    //end Yaw data
}