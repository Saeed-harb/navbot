#include "Odometry.h"

Odometry::Odometry(float wheelRadius, float wheelBase, int leftTicksPerRevolution, int rightTicksPerRevolution)
    : wheelRadius(wheelRadius), wheelBase(wheelBase), x(0), y(0), theta(0), lastLeftCount(0), lastRightCount(0) {
        leftDistancePerTick = (2 * M_PI * wheelRadius) / leftTicksPerRevolution;
        rightDistancePerTick = (2 * M_PI * wheelRadius) / rightTicksPerRevolution;
}
      
void Odometry::update(long leftCount, long rightCount) {
    long deltaLeft = leftCount - lastLeftCount;
    long deltaRight = rightCount - lastRightCount;
    lastLeftCount = leftCount; 
    lastRightCount = rightCount;

    // Convert ticks to distance using pre-calculated values
    float dLeft = deltaLeft * leftDistancePerTick;
    float dRight = deltaRight * rightDistancePerTick;
    float dCenter = (dLeft + dRight) / 2.0;
    float dTheta = (dRight - dLeft) / wheelBase;
    x += dCenter * cos(theta + dTheta / 2);
    y += dCenter * sin(theta + dTheta / 2);
    theta += dTheta; 
    while (theta > M_PI)
        theta -= 2 * M_PI;
    while (theta < -M_PI)
        theta += 2 * M_PI;
}

float Odometry::getX() {
    return x;
}

float Odometry::getY() {
    return y;
}

float Odometry::getTheta() {
    return theta;
}
