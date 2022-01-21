#ifndef Drone_Funcs
#define Drone_Funcs

#include <Adafruit_Sensor.h>

typedef struct Quad {
    double roll;
    double pitch;
    double yaw;
} Quad;

void calculateResponse(sensors_event_t event, Quad *error, Quad *pos);
void writeToESCsRoll(double error, double roll);
void readContData();
void normalizeInput(sensors_event_t event);
void calibrate(Adafruit_BNO055 *bno, sensors_event_t event);
void writeToESCsPitch(double error, double pitch);
void writeToESCsYaw(double error, double yaw);
void initESCs();
void testSD();
void initBNO055(Adafruit_BNO055 *bno);
void initPIDs();

#endif