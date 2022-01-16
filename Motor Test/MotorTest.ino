#include <Servo.h>

int sensorValue, mappedValue;
Servo esc5, esc6, esc7, esc8;
#define SAMPLERATE_DELAY_MS (100)

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
    Serial.begin(9600);
}

void loop()
{
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

    delay(SAMPLERATE_DELAY_MS);
}
