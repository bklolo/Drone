#ifndef drone_defs
#define drone_defs

#include <Adafruit_BNO055.h>

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
void initESCs(int D4, int D5, int D6, int D7);
void testSD();
void initBNO055(Adafruit_BNO055 *bno);
void initPIDs();
void testMotors(uint8_t pin, int delay_ms, long in_min, long in_max, long out_min, long out_max);

#endif