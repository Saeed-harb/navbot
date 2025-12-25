#ifndef ENCODERCLASS_H
#define ENCODERCLASS_H

#include <Arduino.h>

class EncoderClass
{
public:
  EncoderClass(uint8_t pinA, uint8_t pinB);

  void countPulsesLowResolution();

  void countPulsesHighResolution();

  void initEncoder(void (*ISR_callback)(void), uint8_t resolutionEncoder);

  long getPulseCount();

private:
    volatile long totalPulses;
    uint8_t pinEncoderA;
    uint8_t pinEncoderB;
    
}; // class EncoderClass

#endif
