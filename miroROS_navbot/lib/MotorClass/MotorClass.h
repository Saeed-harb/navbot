#ifndef MOTORCLASS_H
#define MOTORCLASS_H

#include <Arduino.h>

class MotorClass
{
public:
    MotorClass(uint8_t motorDp1, uint8_t motorDp2, uint8_t motorP, int ticksPerRevolution);

    void MotorInitialize();

    void MotorControl(int pwmV);

    int getMotorRPM(volatile long currentPulses);

    float getMotorPosition(volatile long currentPulses);

private:
    uint8_t motorDirectionPin1;
    uint8_t motorDirectionPin2;
    uint8_t motorPWMPin;

    int rpm;
    float pulsesPerRevolution;
    // float rpmFilt;
    // float rpmPrev;
    long interval;
    unsigned long previousMillis;
    volatile long previousPulses;

}; // class MotorClass

#endif
