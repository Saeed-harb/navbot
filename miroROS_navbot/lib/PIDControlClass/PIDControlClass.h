#ifndef PIDCONTROLCLASS_H
#define PIDCONTROLCLASS_H

#include <Arduino.h>

class PIDControlClass
{
public:
    PIDControlClass(float kP, float kI, float kD, float delayP);

    // void PIDControllerPositionInitialize(float dA, long AngleMeasurement);
    
    // template <typename T>
    void PIDControllerInitialize(int target, int feedback);

    // template <typename T>
    int ComputeControlAction(int target, int feedback);

private:
    float setPoint;

    float Kp;
    float Ki;
    float Kd;

    float h;

    float errorK;
    float errorKm1;
    float errorKm2;
    float controlK;
    float controlKm1;

    float a1;
    float a2;
    float a3;

}; // class PIDControlClass

#endif
