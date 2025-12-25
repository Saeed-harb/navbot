#include "PIDControlClass.h"

PIDControlClass::PIDControlClass(float kP, float kI, float kD, float delayP)
{
    setPoint= 0;

    Kp = kP;
    Ki = kI;
    Kd = kD;

    h = delayP;

    errorK = 0.0;
    errorKm1 = 0.0;
    errorKm2 = 0.0;
    controlK = 0.0;
    controlKm1 = 0.0;

    a1 = Kp + Ki*h + Kd/h;
    a2 = -(Kp + 2*Kd/h);
    a3 = Kd/h;
}
// template <typename T>
void PIDControlClass::PIDControllerInitialize(int target, int feedback)
{
    setPoint = target;

    errorK = setPoint - (float)feedback;
    errorKm1 = errorK;
    errorKm2 = errorK;
}

// template <typename T>
int PIDControlClass::ComputeControlAction(int target, int feedback)
{
    setPoint = target;
    errorK = setPoint - (float)feedback;

    // if (abs(errorK) < 3.0)
    // {
    //     controlK = 0;
    // }
    // else
    // {
    //     controlK = controlKm1 + a1*errorK + a2*errorKm1 + a3*errorKm2;

    //     if (controlK > 255.0)
    //     {
    //         controlK = 255.0;
    //     }
    //     if (controlK < -255.0)
    //     {
    //         controlK = -255.0;
    //     }
    // }

    controlK = controlKm1 + a1*errorK + a2*errorKm1 + a3*errorKm2;

    if (controlK > 255.0)
    {
        controlK = 255.0;
    }
    if (controlK < -255.0)
    {
        controlK = -255.0;
    }

    controlKm1 = controlK;
    errorKm2 = errorKm1;
    errorKm1 = errorK;

    return (int)controlK;
}
