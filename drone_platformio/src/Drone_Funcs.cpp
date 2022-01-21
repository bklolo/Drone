#include <Arduino.h>
#include "Drone_Funcs.h"
#include <Servo.h> // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <ArduPID.h>
#include <SD.h>
#include <SPI.h>

double setpoint = 0;
double xoffset, yoffset, zoffset = 0;
double iRoll, oRoll;
double iPitch, oPitch;
double iYaw, oYaw;
double p = 1;
double i = 0;
double d = 0;
float adjustedYaw;
int sensorValue, mappedValue, incomingByte[8];
ArduPID rollController, pitchController, yawController;
Servo esc5, esc6, esc7, esc8;
File myFile;

void calculateResponse(sensors_event_t event, Quad *error, Quad *pos)
{
    //Roll data
    pos->roll = event.orientation.z - zoffset;
    iRoll = abs(pos->roll) * -1; // roll data only
    rollController.compute();

    Serial.print("Roll: ");
    Serial.print(pos->roll, 4);
    Serial.print(',');
    Serial.print("Roll error: ");
    Serial.println(oRoll);
    // end Roll data

    // Pitch data
    pos->pitch = event.orientation.y - yoffset;
    iPitch = abs(pos->pitch) * -1;
    pitchController.compute();

    Serial.print("Pitch: ");
    Serial.print(pos->pitch, 4);
    Serial.print(',');
    Serial.print("Pitch error: ");
    Serial.println(oPitch);
    // end Pitch data

    // Yaw data
    pos->yaw = event.orientation.x - xoffset;
    float adjustedYaw = pos->yaw;

    if (pos->yaw > 180)
    {
        adjustedYaw = (adjustedYaw - 360) * -1;
    }

    iYaw = abs(adjustedYaw) * -1;
    yawController.compute();

    Serial.print("Yaw: ");
    Serial.print(pos->yaw, 4);
    Serial.print(", AdjustedYaw: ");
    Serial.print(adjustedYaw, 4);
    Serial.print(',');
    Serial.print("Yaw error: ");
    Serial.println(oYaw);

    error->roll = oRoll;
    error->pitch = oPitch;
    error->yaw = oYaw;

    delay(1000);
}

void writeToESCsRoll(double error, double roll)
{

    mappedValue = map(error, 0, 255, 1000, 2000); // Mapping val to min and max //1021?
    Serial.print("roll mapped:");
    Serial.print(mappedValue);
    if (roll > 0)
    {
        esc5.writeMicroseconds(mappedValue); // Write mapped sensor value to esc5
        esc6.writeMicroseconds(mappedValue); // Write mapped sensor value to esc6
    }
    else
    {
        esc7.writeMicroseconds(mappedValue); // Write mapped sensor value to esc7
        esc8.writeMicroseconds(mappedValue); // Write mapped sensor value to esc8
    }
}

void readContData()
{
    // in order to update nano, must unplug xbee from tx/rx
    // motors on red arms CCW, motors on white arms CW (X-Quad formation)
    // if all motors spin at the same velocity, angular momentum = 0
    // for example: CCW motors = 2, CW motors = -2; 2+2-2-2=0
    // rotation occurs when you decrease spin one set and increase the other
    // to go forward, increase back motors spin and increase front
    // to go backward, increase front motors spin and decrease back
    // ALL ADJUSTMENTS MUST BE SYMMETRICAL

    int n = 0;
    char breakVal = '^';
    char curVal;

    do
    {
        if (Serial.available())
        {
            curVal = Serial.read() + 64;
            incomingByte[n++] = curVal;
        }
        else
        {
            break;
        }

    } while (curVal != breakVal);

    n = 0;
    while (n < 8)
    {
        Serial.write(incomingByte[n++]);
    }

    Serial.print("\n");
}

void normalizeInput(sensors_event_t event)
{
    xoffset = event.orientation.x;
    yoffset = event.orientation.y;
    zoffset = event.orientation.z;
}

void calibrate(Adafruit_BNO055 *bno, sensors_event_t event)
{
    uint8_t sys, gyro, accel, mag;
    sys = gyro = accel = mag = 0;
    bno->getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("Sys:");
    Serial.print(sys, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);

    // print sensor info
    Serial.print("Yaw: ");
    Serial.print(event.orientation.x - xoffset, 4);
    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y - yoffset, 4);
    Serial.print("\tRoll: ");
    Serial.print(event.orientation.z - zoffset, 4);
    Serial.println("");
    delay(500);
}

void writeToESCsPitch(double error, double pitch)
{
    mappedValue = map(error, 0, 255, 1000, 2000);
    Serial.print(", pitch mapped: ");
    Serial.print(mappedValue);
    if (pitch > 0)
    {
        esc6.writeMicroseconds(mappedValue);
        esc7.writeMicroseconds(mappedValue);
    }
    else
    {
        esc5.writeMicroseconds(mappedValue);
        esc8.writeMicroseconds(mappedValue);
    }
}

void writeToESCsYaw(double error, double yaw)
{
    mappedValue = map(error, 0, 255, 1000, 2000);
    Serial.print(", Yaw mapped: ");
    Serial.println(mappedValue);
    if (yaw < 180)
    {
        esc5.writeMicroseconds(mappedValue);
        esc7.writeMicroseconds(mappedValue);
    }
    else
    {
        esc6.writeMicroseconds(mappedValue);
        esc8.writeMicroseconds(mappedValue);
    }

    //sensorValue = analogRead(A0);                       // Read input from analog pin a0 and store in val
    mappedValue = map(sensorValue, 0, 1023, 1000, 2000); // Mapping val to min and max //1021?
}

void initESCs()
{
    // Specify ESC pins (D5-D8) and initialize to 1000
    esc5.attach(5);
    esc5.writeMicroseconds(1000);
    esc6.attach(6);
    esc6.writeMicroseconds(1000);
    esc7.attach(7);
    esc7.writeMicroseconds(1000);
    esc8.attach(8); // pin 8 used for SPI
    esc8.writeMicroseconds(1000);
}

void testSD()
{
    /////////////// SD CODE BEGIN ///////////////
    const int SD_pin = 8;

    pinMode(SD_pin, OUTPUT);

    if (!SD.begin(8))
    {
        Serial.println("initialization failed!");
        return;
    }

    Serial.println("initialization done.");
    myFile = SD.open("data.txt", FILE_WRITE);
    myFile.println("asdfasdfasdfasfasdfasdf");
    myFile.close();
    Serial.print("File is good...");
    /////////////// SD CODE END ///////////////
}

void initBNO055(Adafruit_BNO055 *bno)
{
    /* Initialise the sensor */
    if (!bno->begin())
    {
        Serial.print("No BNO055 detected!");
        while (true)
            ; // end; do nothing
    }
    delay(100);
    bno->setExtCrystalUse(true);
}

void initPIDs()
{
    rollController.begin(&iRoll, &oRoll, &setpoint, p, i, d);
    pitchController.begin(&iPitch, &oPitch, &setpoint, p, i, d);
    yawController.begin(&iYaw, &oYaw, &setpoint, p, i, d);
}

void testMotors(int delay_ms, uint8_t pin, long in_min, long in_max, long out_min, long out_max){
    int sensorValue = analogRead(A0);
    mappedValue = map(sensorValue, 0, 1024, 1000, 2000);

    esc5.writeMicroseconds(mappedValue);
    esc6.writeMicroseconds(mappedValue);
    esc7.writeMicroseconds(mappedValue);
    esc8.writeMicroseconds(mappedValue);

    Serial.print("Pot value: ");
    Serial.print(sensorValue);
    Serial.print(",Mapped value: ");
    Serial.println(mappedValue);

    delay(delay_ms);
}