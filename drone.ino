#include <Servo.h> // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "ArduPID.h"

struct Quad
{
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
};

Quad quad;
// ArduPID vars
ArduPID rollController;
ArduPID pitchController;
ArduPID yawController;
double setpoint = 0;
double iRoll;
double oRoll;
double iPitch;
double oPitch;
double iYaw;
double oYaw;
double p = 1;
double i = 0;
double d = 0;
// end ArduPID vars

Servo esc5, esc6, esc7, esc8;              // ESCs, respective pinout
Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU instance
int sensorValue, mappedValue;              // pot vals
int incomingByte[8];                       // incoming serial data
uint8_t sys, gyro, accel, mag;
float xoffset, yoffset, zoffset = 0;
bool isCalibrated = false;

void setup()
{
  
  sys = gyro = accel = mag = 0;
  // Specify ESC signal (D5-D8) and initialize to 1000
  esc5.attach(5);
  esc5.writeMicroseconds(1000);
  esc6.attach(6);
  esc6.writeMicroseconds(1000);
  esc7.attach(7);
  esc7.writeMicroseconds(1000);
  esc8.attach(8);
  esc8.writeMicroseconds(1000);

  Serial.begin(9600); // baudrate
  rollController.begin(&iRoll, &oRoll, &setpoint, p, i, d);
  pitchController.begin(&iPitch, &oPitch, &setpoint, p, i, d);
  yawController.begin(&iYaw, &oYaw, &setpoint, p, i, d);

  /////////////// BNO CODE BEGIN ///////////////
  /* Initialise the sensor */
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected!");
    while (true)
      ; // end; do nothing
  }
  delay(100);
  bno.setExtCrystalUse(true);

  /////////////// BNO CODE END ///////////////
}

void loop()
{
  // Get a new BNO sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  if (isCalibrated)
  {
    calculateResponse(event);
    writeToESCsRoll(oRoll, quad.roll);
    writeToESCsPitch(oPitch, quad.pitch);
    writeToESCsYaw(oYaw, quad.yaw);
  }
  else
  {
    calibrate(event);
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

void  calculateResponse(sensors_event_t event)
{
  //Roll data
  quad.roll = event.orientation.z - zoffset;
  iRoll = abs(quad.roll)* -1;  // roll data only
  rollController.compute();
  
  Serial.print("Roll: ");
  Serial.print(quad.roll, 4);
  Serial.print(',');
  Serial.print("Roll error: ");
  Serial.println(oRoll);
  // end Roll data

  // Pitch data
  quad.pitch = event.orientation.y -yoffset;
  iPitch = abs(quad.pitch) * -1;
  pitchController.compute();
  
  Serial.print("Pitch: ");
  Serial.print(quad.pitch, 4);
  Serial.print(',');
  Serial.print("Pitch error: ");
  Serial.println(oPitch);
  // end Pitch data

  // Yaw data
  quad.yaw = event.orientation.x - xoffset;
  float adjustedYaw = quad.yaw;

  if(quad.yaw >180)
  {
    adjustedYaw = (adjustedYaw - 360) * -1;
  }

  iYaw = abs(adjustedYaw) * -1;
  yawController.compute();
  
  Serial.print("Yaw: ");
  Serial.print(quad.yaw, 4);
  Serial.print(", AdjustedYaw: ");
  Serial.print(adjustedYaw, 4);
  Serial.print(',');
  Serial.print("Yaw error: ");
  Serial.println(oYaw);
  
  delay(1000);
}

void normalizeInput(sensors_event_t event)
{
  xoffset = event.orientation.x;
  yoffset = event.orientation.y;
  zoffset = event.orientation.z;
}

void calibrate(sensors_event_t event)
{
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
