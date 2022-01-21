#include <Servo.h> // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduPID.h>
#include <SD.h>
#include <SPI.h>
#include "Drone_Funcs.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU instance
bool isCalibrated = false;

/*Testing Motors*/
bool testingMotors = false;
int delay_ms = 100;
uint8_t pin = 'A0';
long in_min = 0;
long in_max = 1024;
long out_min = 1000;
long out_max = 2000;
/*Testing Motors*/

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
    if(testingMotors){
        testMotors(delay_ms, pin, in_min, in_max, out_min, out_max);
    }
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

    if (Serial.available() > 0)
    {
        if (Serial.read() == '0')
        {
            normalizeInput(event);
            Serial.println("Outputs have been normalized!");
        }
    }

    //readContData();   // get serial data from HOTAS controller

}