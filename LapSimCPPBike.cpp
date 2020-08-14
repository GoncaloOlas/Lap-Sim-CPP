#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <algorithm>
#include <thread>
#include <fstream>

//Simulation parameters
float timeStep = 0.001; //seconds
float timer = 0.0;
float GtoGRadius = 2.0;
float StateOfCharge = 20.0; // Ah

//enviromental parameters
float gravity = 9.81;
float rho = 1.162;

//Track parameters
float segment1 = 10.0;
float segment2 = 10.0;
float turn1Rad = 50.0;
float turn2Rad = 100.0;
float perimeter1 = 2 * M_PI *turn1Rad;
float perimeter2 = 2 * M_PI *turn2Rad;
float dd = 1.0;

//vehicle parameters
float mass = 230.0 + 68.0; //Car + driver
float frictionCoef = 1.0;
float muLong = 1.4;
float dragCoef = 0.5;
float liftCoef = 0.12;
float frontalArea = 1.02;
float tireRadius = 0.2286;
float finalDriveRatio = 3.38;
float accVoltage = 399.0;
float rpmCoef = 470.0 / 5170.0;
float maxRpmMotor = accVoltage / rpmCoef;
float tirePerimeter = 2 * M_PI * tireRadius;
float CG_long = 0.45;  // Rear Weight distribution
float CP_long = 0.54;  // percent rear
float CG_vert = 0.300;
float WheelBase = 1.56;
float FrontTrack = 1.180;
float RearTrack = 1.160;
float RollingDrag = 100.0; // For accuracy should be changed to 200 + (250 * pow(LateralGs, 2.0))
float RMSCurrent = 172.5;
float PFactor = 0.94;
float MotorPhaseResistance = 0.007;
float MaxCurrent = 350.0;
float cellResistance = 0.003;
float AccCellNr = 95.0;
float AccResistance = cellResistance * AccCellNr;
float DrivetrainEffic = 0.85;
float BatteryCap = 20.0; // Ah


float motorTorque = 0.0;
float rpmTire = 0.0;
float velocity = 0.0;
float distance = 0.0;
/*
int Cornering(float segment1, float segment2, float turn1Rad, float turn2Rad)
{
  float perimeter1 = 2 * M_PI * turn1Rad;
  while(distance <= perimeter1){
    if(rpmTire < maxRpmMotor){
      rpmTire = (velocity /tirePerimeter) * 60 * finalDriveRatio;
    }else{
      rpmTire = maxRpmMotor;
    }
    motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    if(motorTorque > 240){
      motorTorque = 240;
    }else{
      motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    }
    float downForce = 0.5 * rho * frontalArea * liftCoef * (velocity*velocity);
    float dragForce = 0.5 * rho * frontalArea * dragCoef * (velocity*velocity);
    float wheelTorque = motorTorque * finalDriveRatio;
    float wheelForce = wheelTorque / tireRadius;

    float normalFrontForce = (mass * gravity * (1 - CG_long)) + (downForce * (1 - CP_long));
    float normalRearForce = (mass * gravity * CG_long) + (downForce * CP_long);
    float normalForceTotal = normalFrontForce + normalRearForce;

    float MaxLatTireForce = normalForceTotal * frictionCoef;

    float MaxCornerSpeed = sqrt(MaxLatTireForce / mass * turn1Rad);
    float MaxLatVelocity = mass * (velocity * velocity) / turn1Rad;
    float LatForce = std::min(MaxCornerSpeed, MaxLatVelocity);
    float LatAccel = LatForce / mass;

    float forceFrictionLimitLong = sqrt((1-(LatForce / MaxLatTireForce)) * (1-(LatForce / MaxLatTireForce))
    * ((normalRearForce * muLong) * (normalRearForce * muLong)));
    float ForceLongCP = std::min(forceFrictionLimitLong, wheelForce);
    float ForceLongNet = ForceLongCP - dragForce;
    float LongAccel = ForceLongNet / mass;
    std::cout<<"Longitudinal Accel: "<<LongAccel<<std::endl;
    if(MaxCornerSpeed < sqrt((velocity *velocity) * 2 * LongAccel * dd))
    {
      velocity = MaxCornerSpeed;
      LatAccel = 0;
    }else
    {
      velocity = sqrt((velocity *velocity) * 2 * LongAccel * dd);
    }
    velocity += (LongAccel * timeStep);
    distance += (velocity * timeStep);
    timer += timeStep;
    std::cout<<distance<<std::endl;
  }
  return timer;
}
*/
int straightLine(float finalDistance, float &timer, float &StateOfCharge)
{
  while(finalDistance >= distance)
  {
    float MechMotorPower = (accVoltage / sqrt(2.0)) * sqrt(3.0) * RMSCurrent * PFactor;
    float PowerLossCtrl = (0.00554 * pow(accVoltage, 0.85029) * RMSCurrent) + 211.5;
    float MotorPowerLoss = pow(RMSCurrent, 2.0) * MotorPhaseResistance;
    float AccPowerLoss = pow(RMSCurrent, 2.0) * AccResistance;
    float motorPower = MechMotorPower - PowerLossCtrl - MotorPowerLoss - AccPowerLoss;
    if(rpmTire < maxRpmMotor)
    {
      rpmTire = (velocity /tirePerimeter) * 60 * finalDriveRatio;
    }else
    {
      rpmTire = maxRpmMotor;
    }
    motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    if(motorTorque > 230.0)
    {
      motorTorque = 230.0;
    }else
    {
      motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    }
    float wheelTorque = motorTorque * finalDriveRatio * DrivetrainEffic;
    float wheelForce = wheelTorque / tireRadius;

    float downForce = 0.5 * rho * frontalArea * liftCoef * pow(velocity,2.0);
    float dragForce = 0.5 * rho * frontalArea * dragCoef * pow(velocity, 2.0);

    float normalFrontForce = (mass * gravity * (1 - CG_long)) + (downForce * (1 - CP_long));
    float normalRearForce = (mass * gravity * CG_long) + (downForce * CP_long);
    float normalForceTotal = normalFrontForce + normalRearForce;

    float frictionForce = frictionCoef * normalForceTotal;
    float motorExactPower = motorPower / velocity;

    float realMotorForce = std::min(wheelForce, motorExactPower);
    float contactPatchForce = std::min(realMotorForce, frictionForce);

    float accelerationPowerLimited = (contactPatchForce - dragForce) / mass;

    float accelerationGripLimited = (-WheelBase * (((mass * gravity * CG_long) + downForce) * muLong) - (RollingDrag + dragForce)) / (mass * (muLong * CG_vert - WheelBase));

    float acceleration = std::min(accelerationPowerLimited, accelerationGripLimited);

    velocity += (acceleration * timeStep);
    distance += (velocity * timeStep);
    timer += timeStep;
    StateOfCharge -= 0.0277 * timeStep;
  }
  return 0;
}

int main()
{
  auto t1 = std::chrono::high_resolution_clock::now();
  int mode = 1; // 1 == accel run, 2 == track run
  switch(mode)
  {
    case 1:
    {
      float finalDistance = 75.0;
      straightLine(finalDistance, timer, StateOfCharge);
      break;
    }
    case 2:
    {
      //Track parameters
      float segment1 = 500.0;
      float segment2 = 500.0;
      float turn1Rad = 9.125;
      float turn2Rad = 9.125;

      //Cornering(segment1, segment2, turn1Rad, turn2Rad);
      break;
    }
  }
  std::cout<<"Accel time: "<<timer<<std::endl;
  std::cout<<"Accumulator Capacity: "<<StateOfCharge * accVoltage<<std::endl;
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"Program Run time: "<<duration/1000<<std::endl; // milliseconds
  return 0;
}
