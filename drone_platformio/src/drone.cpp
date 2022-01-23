#include <Servo.h> // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ArduPID.h>
#include <SD.h>
#include <SPI.h>
#include "drone_defs.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU instance
bool isCalibrated = false;

/*ESC pins*/
int D4 = 4;
int D5 = 5;
int D6 = 6;
int D7 = 7;
/*ESC pins*/

/*Testing Motors*/
bool testingMotors = false;
uint8_t pin = A0;
int delay_ms = 100;
long in_min = 0;
long in_max = 1024;
long out_min = 1000;
long out_max = 2000;
/*Testing Motors*/

/*Testing SD & BNO*/
bool testingSD = true;
/*Testing SD & BNO*/

void setup()
{
    Serial.begin(9600); // default baudrate
    initESCs(D4, D5, D6, D7);
    testSD();
    delay(500);
    initBNO055(&bno);
    initPIDs();
}

void loop()
{
    // test motors with onboard potentiometer
    while(testingMotors){
        testMotors(pin, delay_ms, in_min, in_max, out_min, out_max);
    }

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