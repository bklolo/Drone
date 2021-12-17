#include <Servo.h>                          // Servo library to control ESC
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Servo esc5,esc6,esc7,esc8;                  // ESCs, respective pinout
Adafruit_BNO055 bno = Adafruit_BNO055(55);  // IMU instance
int sensorValue, mappedValue;               // pot vals
int incomingByte[8];                        // incoming serial data
int xValue = 0;
int incomingByte0 = 0; // for incoming serial data


void setup()
{
  // Specify ESC signal (D5-D8) and initialize to 1000
  esc5.attach(5);
  esc5.writeMicroseconds(1000);
  esc6.attach(6); 
  esc6.writeMicroseconds(1000); 
  esc7.attach(7); 
  esc7.writeMicroseconds(1000); 
  esc8.attach(8); 
  esc8.writeMicroseconds(1000); 
 
  Serial.begin(9600);   // baud of 9600
  
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
  // in order to update nano, must unplug xbee from tx/rx
  // motors on red arms CCW, motors on white arms CW (X-Quad formation)
  // if all motors spin at the same velocity, angular momentum = 0
  // for example: CCW motors = 2, CW motors = -2; 2+2-2-2=0
  // rotation occurs when you decrease spin one set and increase the other
  // to go forward, increase back motors spin and increase front
  // to go backward, increase front motors spin and decrease back
  // ALL ADJUSTMENTS MUST BE SYMMETRICAL

// Purge any old data
  while (Serial.available()) 
  {
    Serial.read();
  }

  int n = 0;
  uint32_t start = millis();
  
  while (n < 8) {
    // 1 second timeout
    if (millis() - start > 1000)
    {
//      Serial.print("break");
        break;
    }

    if (Serial.available()) {
        incomingByte[n++] = Serial.read();
    }
    else {
      Serial.print("");
    }
  }

  n = 0;
  while(n < 8)
  {
    Serial.print(incomingByte[n++]);
  }
  
  Serial.println("");

  sensorValue = analogRead(A0);                       // Read input from analog pin a0 and store in val
  mappedValue = map(sensorValue, 0, 1023,1000,2000);  // Mapping val to min and max //1021?
  esc5.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc5
  esc6.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc6
  esc7.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc7
  esc8.writeMicroseconds(mappedValue);                // Write mapped sensor value to esc8

  // Get a new BNO sensor event
  sensors_event_t event;
  bno.getEvent(&event);
  
  /* Display the floating point data 
  Serial.print("Yaw: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tPitch: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tRoll: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
*/
  delay(500);

}
