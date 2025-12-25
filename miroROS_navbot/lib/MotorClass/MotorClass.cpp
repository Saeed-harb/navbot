#include "MotorClass.h"

MotorClass::MotorClass(uint8_t motorDp1, uint8_t motorDp2, uint8_t motorP, int ticksPerRevolution)
{
    motorDirectionPin1 = motorDp1;
    motorDirectionPin2 = motorDp2;
    motorPWMPin = motorP;
    previousMillis = 0;
    previousPulses = 0;
    interval = 100;
    rpm = 0;
    pulsesPerRevolution = ticksPerRevolution;
    // rpmFilt = 0;
}

void MotorClass::MotorInitialize()
{
    pinMode(motorDirectionPin1, OUTPUT);
    pinMode(motorDirectionPin2, OUTPUT);
    pinMode(motorPWMPin, OUTPUT);

    digitalWrite(motorDirectionPin1, LOW);
    digitalWrite(motorDirectionPin2, LOW);
}

void MotorClass::MotorControl(int pwmV)
{
    int directionM;

    if (pwmV >= 0)
    {
        directionM = 1;
    }
    else
    {
        directionM = -1;
    }

    if (directionM == 1)
    {
        analogWrite(motorPWMPin, (uint8_t)abs(pwmV));
        digitalWrite(motorDirectionPin1, HIGH);
        digitalWrite(motorDirectionPin2, LOW);
    }

    if (directionM == -1)
    {
        analogWrite(motorPWMPin, (uint8_t)abs(pwmV));
        digitalWrite(motorDirectionPin1, LOW);
        digitalWrite(motorDirectionPin2, HIGH);
    }
}

int MotorClass::getMotorRPM(volatile long currentPulses)
{
    unsigned long currentMillis = millis();
    volatile long pulses = currentPulses - previousPulses;

    if ((currentMillis - previousMillis) >= interval)
    {
        previousMillis = currentMillis;
        rpm = 10*pulses*(60.0/224.0);
        // rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev; // Low-pass filter (25 Hz cutoff)
        // rpmPrev = rpm;
        previousPulses = currentPulses;
    }

    return rpm;
}

float MotorClass::getMotorPosition(volatile long currentPulses)
{
    volatile long pulses = currentPulses;

    float revolutions = pulses / pulsesPerRevolution;
    float position = revolutions * 6.28;

    return position;
}
