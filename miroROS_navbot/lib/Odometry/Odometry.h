#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <math.h>

class Odometry
{
public:
    Odometry(float wheelRadius, float wheelBase, int leftTicksPerRevolution, int rightTicksPerRevolution);

    void update(long leftCount, long rightCount);
    float getX();
    float getY();
    float getTheta();

private:
    float wheelRadius;
    float wheelBase;
    float x;
    float y;
    float theta;
    long lastLeftCount;
    long lastRightCount;
    float leftDistancePerTick;
    float rightDistancePerTick;
}; // class Odometry


#endif
