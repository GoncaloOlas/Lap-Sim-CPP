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

constexpr float calcMaxCornerSpeed(float Turn1Radius, float WingArea, float WingDragCoef) {
    return std::sqrt((frictionCoef * mass * gravity) / ((std::sqrt(std::pow(mass / Turn1Radius, 2.0) + std::pow((0.5 * rho * ((frontalArea * dragCoef) + (WingArea * WingDragCoef))), 2.0))) - (0.5 * frictionCoef * rho * WingArea * liftCoef)));
}

constexpr float calcAccelerationGripLimited(float downForce, float velocity) {
    return (-WheelBase * (((mass * gravity * CG_long) + downForce) * muLong) - (RollingDrag + calcDragForce(velocity))) / (mass * (muLong * CG_vert - WheelBase));
}

constexpr float calculateMotorPower(float accVoltage, float RMSCurrent, float PFactor)
{
    return (accVoltage / std::sqrt(2.0)) * std::sqrt(3.0) * RMSCurrent * PFactor;
}

constexpr float calculateTotalPowerLoss(float accVoltage, float RMSCurrent)
{
    return (0.00554 * std::pow(accVoltage, 0.85029) * RMSCurrent) + 211.5;
}

constexpr float calculateMotorPowerLoss(float RMSCurrent, float MotorPhaseResistance)
{
    return std::pow(RMSCurrent, 2.0) * MotorPhaseResistance;
}

constexpr float calculateAccPowerLoss(float RMSCurrent, float AccResistance)
{
    return std::pow(RMSCurrent, 2.0) * AccResistance;
}

#endif // UTILS_HPP
