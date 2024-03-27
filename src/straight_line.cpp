#include "straight_line.hpp"

int straightLine(float finalDistance, float &timer, float &StateOfCharge, float &velocity) {
    float distanceStraight = 0.0;

    std::ofstream vel_writer("velocity.txt");
    std::ofstream accel_writer("acceleration.txt");
    std::ofstream torque_writer("torque.txt");
    std::ofstream force_writer("normal_force.txt");

    float rpmTire = 0.0;
    float motorTorque = 0.0;
    while (finalDistance >= distanceStraight) {
        float MechMotorPower = (accVoltage / std::sqrt(2.0)) * std::sqrt(3.0) * RMSCurrent * PFactor;
        float CtrlPowerLoss = (0.00554 * std::pow(accVoltage, 0.85029) * RMSCurrent) + 211.5;
        float MotorPowerLoss = std::pow(RMSCurrent, 2.0) * MotorPhaseResistance;
        float AccPowerLoss = std::pow(RMSCurrent, 2.0) * AccResistance;
        float motorPower = MechMotorPower - CtrlPowerLoss - MotorPowerLoss - AccPowerLoss;

        if (rpmTire < maxRpmMotor)
            rpmTire = (velocity / tirePerimeter) * 60 * finalDriveRatio;
        else
            rpmTire = maxRpmMotor;

        motorTorque = motorPower / ((rpmTire * 2 * _USE_MATH_DEFINES) / 60);
        if (motorTorque > 230.0)
            motorTorque = 230.0;
        else
            motorTorque = motorPower / ((rpmTire * 2 * _USE_MATH_DEFINES) / 60);

        float wheelTorque = calcWheelTorque(motorPower, rpmTire);
        float wheelForce = wheelTorque / tireRadius;

        float downForce = calcDownForce(velocity);
        float dragForce = calcDragForce(velocity);

        float normalFrontForce = calcNormalFrontForce(downForce);
        float normalRearForce = calcNormalRearForce(downForce);
        float normalForceTotal = calcNormalForceTotal(normalFrontForce, normalRearForce);

        float frictionForce = frictionCoef * normalForceTotal;
        float motorExactPower = motorPower / velocity;

        float realMotorForce = std::min(wheelForce, motorExactPower);
        float contactPatchForce = std::min(realMotorForce, frictionForce);

        float accelerationPowerLimited = calcAcceleration(contactPatchForce, dragForce);
        float accelerationGripLimited = calcAccelerationGripLimited(downForce, velocity);

        float acceleration = std::min(accelerationPowerLimited, accelerationGripLimited);

        velocity += (acceleration * timeStep);
        distanceStraight += (velocity * timeStep);
        timer += timeStep;
        StateOfCharge -= 0.0277 * timeStep;

        vel_writer << velocity * 3.6 << "km/h" << std::endl;
        accel_writer << acceleration << "m/s^2" << std::endl;
        torque_writer << wheelTorque << "N.m" << std::endl;
        force_writer << normalForceTotal << "N" << std::endl;
    }

    vel_writer.close();
    accel_writer.close();
    torque_writer.close();
    force_writer.close();

    return 0;
}