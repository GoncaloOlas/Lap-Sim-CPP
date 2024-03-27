#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <fstream>

constexpr float calcWheelTorque(float motorPower, float rpmTire) {
    return motorPower / ((rpmTire * 2 * _USE_MATH_DEFINES) / 60);
}

constexpr float calcDragForce(float velocity) {
    return 0.5 * rho * frontalArea * dragCoef * std::pow(velocity, 2.0);
}

constexpr float calcDownForce(float velocity) {
    return 0.5 * rho * frontalArea * liftCoef * std::pow(velocity, 2.0);
}

constexpr float calcNormalFrontForce(float downForce) {
    return (mass * gravity * (1 - CG_long)) + (downForce * (1 - CP_long));
}

constexpr float calcNormalRearForce(float downForce) {
    return (mass * gravity * CG_long) + (downForce * CP_long);
}

constexpr float calcNormalForceTotal(float normalFrontForce, float normalRearForce) {
    return normalFrontForce + normalRearForce;
}

constexpr float calcAcceleration(float contactPatchForce, float mass) {
    return contactPatchForce / mass;
}

constexpr float calcMaxCornerSpeed(float frictionCoef, float mass, float gravity, float Turn1Radius, float rho, float frontalArea, float dragCoef, float WingArea, float WingDragCoef, float liftCoef) {
    return std::sqrt((frictionCoef * mass * gravity) / ((std::sqrt(std::pow(mass / Turn1Radius, 2.0) + std::pow((0.5 * rho * ((frontalArea * dragCoef) + (WingArea * WingDragCoef))), 2.0))) - (0.5 * frictionCoef * rho * WingArea * liftCoef)));
}

constexpr float calcAccelerationGripLimited(float downForce, float velocity) {
    return (-WheelBase * (((mass * gravity * CG_long) + downForce) * muLong) - (RollingDrag + calcDragForce(velocity))) / (mass * (muLong * CG_vert - WheelBase));
}


#endif // UTILS_HPP
