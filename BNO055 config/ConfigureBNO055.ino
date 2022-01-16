#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
uint8_t sys, gyro, accel, mag;
float xoffset, yoffset, zoffset = 0;

void setup()
{
    Serial.begin(9600);

    if (!bno.begin())
    {
        Serial.print("No BNO055 detected!");
        while (true)
            ; // end; do nothing
    }
    delay(100);
    bno.setExtCrystalUse(true);
}

void start()
{
    sensors_event_t event;
    bno.getEvent(&event);

    bno.getCalibration(&sys, &gyro, &accel, &mag);
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
    // normalizing
    if (Serial.available() > 0)
    {
        xoffset = event.orientation.x;
        yoffset = event.orientation.y;
        zoffset = event.orientation.z;
    }
}