#include "constants.hpp"
#include "utils.hpp"

void writeOutput(std::ofstream &vel_writer, std::ofstream &accel_writer, std::ofstream &torque_writer, std::ofstream &force_writer, float velocity, float LongAccel, float wheelTorque, float normalForceTotal)
{
    vel_writer << velocity * 3.6 << "km/h" << std::endl;
    accel_writer << LongAccel << "m/s^2" << std::endl;
    torque_writer << wheelTorque << "N.m" << std::endl;
    force_writer << normalForceTotal << "N" << std::endl;
}

int cornering(float Turn1Radius, float SectorLength, float &timer, float &StateOfCharge, float &velocity)
{
    float distanceCorner = 0.0;

    std::ofstream vel_writer("cornering_output/velocity.txt");
    std::ofstream accel_writer("cornering_output/acceleration.txt");
    std::ofstream torque_writer("cornering_output/torque.txt");
    std::ofstream force_writer("cornering_output/normal_force.txt");

    while (distanceCorner <= SectorLength)
    {
        float MechMotorPower = calculateMotorPower(accVoltage, RMSCurrent, PFactor);
        float CtrlPowerLoss = calculateTotalPowerLoss(accVoltage, RMSCurrent);
        float MotorPowerLoss = calculateMotorPowerLoss(RMSCurrent, MotorPhaseResistance);
        // Calculate power losses
        float AccPowerLoss = calculateAccPowerLoss(RMSCurrent, AccResistance);
        
        // Calculate motor power
        float motorPower = MechMotorPower - CtrlPowerLoss - MotorPowerLoss - AccPowerLoss;

        // Calculate rpmTire
        float rpmTire = (velocity / tirePerimeter) * 60 * finalDriveRatio;
        rpmTire = std::min(rpmTire, maxRpmMotor);

        // Calculate wheel torque
        float wheelTorque = calcWheelTorque(motorPower, rpmTire) * finalDriveRatio;
        
        // Calculate down force and drag force
        float downForce = calcDownForce(velocity);
        float dragForce = calcDragForce(velocity);
        
        // Calculate normal forces
        float normalFrontForce = calcNormalFrontForce(downForce);
        float normalRearForce = calcNormalRearForce(downForce);
        float normalForceTotal = calcNormalForceTotal(normalFrontForce, normalRearForce);

        // Calculate maximum lateral tire force
        float MaxLatTireForce = normalForceTotal * frictionCoef;

        // Calculate maximum corner speed
        float MaxCornerSpeed = calcMaxCornerSpeed(Turn1Radius, WingArea, WingDragCoef);
        
        // Calculate lateral force and acceleration
        float MaxLatVelocity = mass * std::pow(velocity, 2.0) / Turn1Radius;
        float LatForce = std::min(MaxCornerSpeed, MaxLatVelocity);
        float LatAccel = LatForce / mass;

        // Calculate longitudinal force and acceleration
        float forceFrictionLimitLong = std::sqrt((1 - (LatForce / MaxLatTireForce)) * (1 - (LatForce / MaxLatTireForce)) * ((normalRearForce * muLong) * (normalRearForce * muLong)));
        float ForceLongCP = std::min(forceFrictionLimitLong, wheelTorque / tireRadius);
        float ForceLongNet = ForceLongCP - dragForce;
        float LongAccel = ForceLongNet / mass;

        // Adjust velocity based on cornering conditions
        if (MaxCornerSpeed < std::sqrt(std::pow(velocity, 2.0) * 2 * LongAccel * SectorLength)) {
            velocity = MaxCornerSpeed;
            LatAccel = 0;
        } else {
            velocity = std::sqrt(std::pow(velocity, 2.0) * 2 * LongAccel * SectorLength);
        }

        // Update simulation parameters
        velocity += (LongAccel * timeStep);
        distanceCorner += (velocity * timeStep);
        timer += timeStep;
        StateOfCharge -= 0.0277 * timeStep;

        // Write output to files
        writeOutput(vel_writer, accel_writer, torque_writer, force_writer, velocity, LongAccel, wheelTorque, normalForceTotal);
    }

    // Close output files
    vel_writer.close();
    accel_writer.close();
    torque_writer.close();
    force_writer.close();

    return 0;
}
