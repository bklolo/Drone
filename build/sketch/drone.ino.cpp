#include <Arduino.h>
#line 1 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
#include <Servo.h>                          // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "ArduPID.h"

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

Servo esc5,esc6,esc7,esc8;                  // ESCs, respective pinout
Adafruit_BNO055 bno = Adafruit_BNO055(55);  // IMU instance
int sensorValue, mappedValue;               // pot vals
int incomingByte[8];                        // incoming serial data
uint8_t sys, gyro, accel, mag;
float xoffset, yoffset, zoffset = 0;

#line 31 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void setup();
#line 62 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void loop();
#line 102 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void readContData();
#line 134 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void calibrate(sensors_event_t event);
#line 162 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void writeToESCsRoll(double error, double roll);
#line 177 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void writeToESCsPitch(double error, double pitch);
#line 192 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
void writeToESCsYaw(double error, double yaw);
#line 31 "c:\\Users\\samfo\\Documents\\Projects\\Drone\\drone.ino"
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
 
  Serial.begin(9600);   // baudrate
  rollController.begin(&iRoll, &oRoll, &setpoint, p, i, d);
  pitchController.begin(&iPitch, &oPitch, &setpoint, p, i, d);
  yawController.begin(&iYaw, &oYaw, &setpoint, p, i, d);
  
/////////////// BNO CODE BEGIN ///////////////
  /* Initialise the sensor */
  if(!bno.begin())
  {
    Serial.print("No BNO055 detected!");
    while(true);  // end; do nothing
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
  
  //calibrate(event);      // calibrate the bno055 
  readContData();   // get serial data from HOTAS controller

  // Roll data
  double roll = event.orientation.z - zoffset;
  iRoll = abs(roll)* -1;  // roll data only
  rollController.compute();
  Serial.print(oRoll);
  Serial.print(',');
  Serial.println(roll, 4);
  writeToESCsRoll(oRoll, roll);
  // end Roll data

  // Pitch data
  double pitch = event.orientation.y -yoffset;
  iPitch = abs(pitch) * -1;
  pitchController.compute();
  Serial.print(oPitch);
  Serial.print(',');
  Serial.println(roll, 4);
  writeToESCsPitch(oPitch, pitch);
  // end Pitch data

  // Yaw data
  double yaw = event.orientation.x - xoffset;
  iYaw = abs(yaw) * -1;
  yawController.compute();
  Serial.print(oYaw);
  Serial.print(',');
  Serial.println(yaw, 4);
  writeToESCsYaw(oYaw, yaw);
  // end Yaw data
  
}
  void readContData(){
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
  
  do{
    if (Serial.available()) {
      curVal = Serial.read() + 64;
        incomingByte[n++] = curVal;
    }
    else {break;}
    
  }while(curVal != breakVal);
  
  n = 0;
  while(n < 8)
  {
    Serial.write(incomingByte[n++]);
  }
  
  Serial.print("\n");
  }

  void calibrate(sensors_event_t event){
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
    Serial.print(event.orientation.y -yoffset, 4);
    Serial.print("\tRoll: ");
    Serial.print(event.orientation.z - zoffset, 4);
    Serial.println("");

    // normalizing
    if (Serial.available() > 0) {
      xoffset = event.orientation.x;
      yoffset = event.orientation.y;
      zoffset = event.orientation.z;
    }
  }

  void writeToESCsRoll(double error, double roll){

    mappedValue = map(error, 0, 255,1000,2000);  // Mapping val to min and max //1021?
    if(roll > 0)
    {
      esc5.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc5
      esc6.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc6
    }
    else
    {
      esc7.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc7
      esc8.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc8
    }
  }

    void writeToESCsPitch(double error, double pitch){
    mappedValue = map(error, 0, 255,1000,2000);
    if(pitch > 0)
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
    
    void writeToESCsYaw(double error, double yaw){
    mappedValue = map(error, 0, 255,1000,2000);
    if(yaw < 180)
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
    mappedValue = map(sensorValue, 0, 1023,1000,2000);  // Mapping val to min and max //1021?
    
  }

