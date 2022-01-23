#include <Arduino.h>
#include "drone_defs.h"
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
Servo motor1, motor2, motor3, motor4;
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

    mappedValue = map(error, 0, 255, 1000, 2000);
    Serial.print("roll mapped:");
    Serial.print(mappedValue);
    if (roll > 0)
    {
        motor1.writeMicroseconds(mappedValue);
        motor2.writeMicroseconds(mappedValue);
    }
    else
    {
        motor3.writeMicroseconds(mappedValue);
        motor4.writeMicroseconds(mappedValue);
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
        motor2.writeMicroseconds(mappedValue);
        motor3.writeMicroseconds(mappedValue);
    }
    else
    {
        motor1.writeMicroseconds(mappedValue);
        motor4.writeMicroseconds(mappedValue);
    }
}

void writeToESCsYaw(double error, double yaw)
{
    mappedValue = map(error, 0, 255, 1000, 2000);
    Serial.print(", Yaw mapped: ");
    Serial.println(mappedValue);
    if (yaw < 180)
    {
        motor1.writeMicroseconds(mappedValue);
        motor2.writeMicroseconds(mappedValue);
    }
    else
    {
        motor3.writeMicroseconds(mappedValue);
        motor4.writeMicroseconds(mappedValue);
    }

    //sensorValue = analogRead(A0);                       // Read input from analog pin a0 and store in val
    mappedValue = map(sensorValue, 0, 1023, 1000, 2000); // Mapping val to min and max //1021?
}

void initESCs(int D4, int D5, int D6, int D7)
{
    // Attach each motor and their respective ESC to digital pin
    motor1.attach(D4);
    motor1.writeMicroseconds(1000);

    motor2.attach(D5);
    motor2.writeMicroseconds(1000);
    
    motor3.attach(D6);
    motor3.writeMicroseconds(1000);
    
    motor4.attach(D7);
    motor4.writeMicroseconds(1000);
}

void testSD()
{
    const int SD_pin = 8;
    String fileName = "data.txt";
    pinMode(SD_pin, OUTPUT);

    if (!SD.begin(SD_pin))
    {
        Serial.println("initialization failed!");
        return;
    }
    Serial.println("initialization done.");
    myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
        Serial.print("Writing to file...");
        myFile.println("testing 1, 2, 3.");
        myFile.close();
        Serial.println("done.");
    } else {
        Serial.println("error opening file"); // if the file didn't open, print an error
    }

    myFile = SD.open("file"); // re-open the file for reading

    if (myFile) {
        Serial.println("file:");
    
    while (myFile.available()) {    // read from the file until there's nothing else in it
        Serial.write(myFile.read());
    }
    myFile.close(); // then close the file
    } else {
        Serial.println("error opening file"); // if the file didn't open, print an error
    }
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

void testMotors(uint8_t pin, int delay_ms, long in_min, long in_max, long out_min, long out_max){
    int sensorValue = analogRead(pin);
    mappedValue = map(sensorValue, 0, 1024, 1000, 2000);

    motor1.writeMicroseconds(mappedValue);
    motor2.writeMicroseconds(mappedValue);
    motor3.writeMicroseconds(mappedValue);
    motor4.writeMicroseconds(mappedValue);

    Serial.print("Pot value: ");
    Serial.println(sensorValue);
    Serial.print("Mapped value: ");
    Serial.println(mappedValue);

    delay(delay_ms);
}