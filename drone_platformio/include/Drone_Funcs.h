#ifndef Drone_Funcs
#define Drone_Funcs

#include <Adafruit_Sensor.h>

void calculateResponse(sensors_event_t event);
void writeToESCsRoll(double error, double roll);
void readContData();
void normalizeInput(sensors_event_t event);
void calibrate(sensors_event_t event);
void writeToESCsPitch(double error, double pitch);
void writeToESCsYaw(double error, double yaw);
void initESCs();
void testSD();
void initBNO055();

struct Quad {
    double roll;
    double pitch;
    double yaw;
};

#endif