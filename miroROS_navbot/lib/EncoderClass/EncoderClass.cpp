#include "EncoderClass.h"

EncoderClass::EncoderClass(uint8_t pinA, uint8_t pinB)
{
    pinEncoderA = pinA;
    pinEncoderB = pinB;
    totalPulses = 0;
}

void IRAM_ATTR EncoderClass::countPulsesLowResolution()
{
    if (digitalRead(pinEncoderB) > 0)
    {
        totalPulses++;
    }
    else
    {
        totalPulses--;
    }
}

void IRAM_ATTR EncoderClass::countPulsesHighResolution()
{
    if (digitalRead(pinEncoderB) != digitalRead(pinEncoderA))
    {
        totalPulses--;
    }
    else
    {
        totalPulses++;
    }
}

long EncoderClass::getPulseCount()
{
    return totalPulses;
}

void EncoderClass::initEncoder(void (*ISR_callback)(void), uint8_t resolutionEncoder)
{
    pinMode(pinEncoderA, INPUT);
    pinMode(pinEncoderB, INPUT);

    if (resolutionEncoder == 1)
    {
        attachInterrupt(digitalPinToInterrupt(pinEncoderA), ISR_callback, RISING);
    }
    if (resolutionEncoder == 2)
    {
        attachInterrupt(digitalPinToInterrupt(pinEncoderA), ISR_callback, CHANGE);
    }
}
