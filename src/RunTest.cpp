#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <algorithm>
#include <thread>
#include <fstream>

int Braking(float Turn1Radius, float SectorLenght, float &timer, float &velocity);
int straightLine(float finalDistance, float &timer, float &StateOfCharge, float &velocity);
int Cornering(float Turn1Radius, float SectorLenght, float &timer, float &StateOfCharge, float &velocity);

//Simulation parameters
float timeStep = 0.001; //seconds
float timer = 0.0;
float StateOfCharge = 20.0; // Ah
float current_distance = 0.0;

//enviromental parameters
float gravity = 9.81;
float rho = 1.162;

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

int Braking(float Turn1Radius, float SectorLenght, float &timer, float &velocity)
{
  float AvailableBrakingForce = (mass * gravity) + (0.5 * rho * liftCoef * pow(velocity,2.0));
  float dragForce = 0.5 * rho * frontalArea * dragCoef * pow(velocity, 2.0);
  float TotalDecelForce = AvailableBrakingForce + dragForce;
  float Deceleration = TotalDecelForce / mass;
  //MaximumEntrySpeed = sqrt(pow(velocity, 2.0) + (2 * Deceleration * SectorLenght));
  
  float WingArea = 0.0;
  float WingDragCoef = 0.0;
  float MaximumEntrySpeed = sqrt((frictionCoef * mass * gravity) / ((sqrt(pow(mass / Turn1Radius, 2.0) + pow((0.5 * rho * ((frontalArea * dragCoef) + (WingArea * WingDragCoef))), 2.0))) - (0.5 * frictionCoef * rho * WingArea * liftCoef)));
  float initial_vel = velocity;

  if(velocity > MaximumEntrySpeed)
  {
    velocity = MaximumEntrySpeed;
  }else
  {
    velocity = velocity;
  }

  float final_vel = velocity;
  float braking_distance = (pow(initial_vel, 2.0) - pow(final_vel, 2.0))/(2*Deceleration);
  timer += (initial_vel - final_vel) / Deceleration;
  return 0;
}

int Cornering(float Turn1Radius, float SectorLenght, float &timer, float &StateOfCharge, float &velocity)
{
  float distanceCorner = 0.0;
  std::ofstream vel_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/cornering_output/velocity.txt");
  std::ofstream accel_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/cornering_output/acceleration.txt");
  std::ofstream torque_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/cornering_output/torque.txt");
  std::ofstream force_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/cornering_output/normal_force.txt");
  while(distanceCorner <= SectorLenght){
    float MechMotorPower = (accVoltage / sqrt(2.0)) * sqrt(3.0) * RMSCurrent * PFactor;
    float CtrlPowerLoss = (0.00554 * pow(accVoltage, 0.85029) * RMSCurrent) + 211.5;
    float MotorPowerLoss = pow(RMSCurrent, 2.0) * MotorPhaseResistance;
    float AccPowerLoss = pow(RMSCurrent, 2.0) * AccResistance;
    float motorPower = MechMotorPower - CtrlPowerLoss - MotorPowerLoss - AccPowerLoss;
    if(rpmTire < maxRpmMotor){
      rpmTire = (velocity /tirePerimeter) * 60 * finalDriveRatio;
    }else{
      rpmTire = maxRpmMotor;
    }
    motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    if(motorTorque > 230){
      motorTorque = 230;
    }else{
      motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    }
    float downForce = 0.5 * rho * frontalArea * liftCoef * pow(velocity, 2.0);
    float dragForce = 0.5 * rho * frontalArea * dragCoef * pow(velocity, 2.0);
    float wheelTorque = motorTorque * finalDriveRatio * DrivetrainEffic;
    float wheelForce = wheelTorque / tireRadius;

    float normalFrontForce = (mass * gravity * (1 - CG_long)) + (downForce * (1 - CP_long));
    float normalRearForce = (mass * gravity * CG_long) + (downForce * CP_long);
    float normalForceTotal = normalFrontForce + normalRearForce;

    float MaxLatTireForce = normalForceTotal * frictionCoef;

    float WingArea = 0.0;
    float WingDragCoef = 0.0;
    float MaxCornerSpeed = sqrt((frictionCoef * mass * gravity) / ((sqrt(pow(mass / Turn1Radius, 2.0) + pow((0.5 * rho * ((frontalArea * dragCoef) + (WingArea * WingDragCoef))), 2.0))) - (0.5 * frictionCoef * rho * WingArea * liftCoef)));
    float MaxLatVelocity = mass * pow(velocity, 2.0) / Turn1Radius;
    float LatForce = std::min(MaxCornerSpeed, MaxLatVelocity);
    float LatAccel = LatForce / mass;

    float forceFrictionLimitLong = sqrt((1-(LatForce / MaxLatTireForce)) * (1-(LatForce / MaxLatTireForce))
    * ((normalRearForce * muLong) * (normalRearForce * muLong)));
    float ForceLongCP = std::min(forceFrictionLimitLong, wheelForce);
    float ForceLongNet = ForceLongCP - dragForce;
    float LongAccel = ForceLongNet / mass;
    if(MaxCornerSpeed < sqrt(pow(velocity, 2.0) * 2 * LongAccel * SectorLenght))
    {
      velocity = MaxCornerSpeed;
      LatAccel = 0;
    }else
    {
      velocity = sqrt(pow(velocity, 2.0) * 2 * LongAccel * SectorLenght);
    }
    velocity += (LongAccel * timeStep);
    distanceCorner += (velocity * timeStep);
    timer += timeStep;
    StateOfCharge -= 0.0277 * timeStep;
    vel_writer<<velocity*3.6<<"km/h"<<std::endl;
    accel_writer<<LongAccel<<"m/s^2"<<std::endl;
    torque_writer<<wheelTorque<<"N.m"<<std::endl;
    force_writer<<normalForceTotal<<"N"<<std::endl;
  }
  vel_writer.close();
  accel_writer.close();
  torque_writer.close();
  force_writer.close();
  return 0;
}

int straightLine(float finalDistance, float &timer, float &StateOfCharge, float &velocity)
{
  float distanceStraight = 0.0;

  std::ofstream vel_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/accel_output/velocity.txt");
  std::ofstream accel_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/accel_output/acceleration.txt");
  std::ofstream torque_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/accel_output/torque.txt");
  std::ofstream force_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/accel_output/normal_force.txt");

  while(finalDistance >= distanceStraight)
  {
    float MechMotorPower = (accVoltage / sqrt(2.0)) * sqrt(3.0) * RMSCurrent * PFactor;
    float CtrlPowerLoss = (0.00554 * pow(accVoltage, 0.85029) * RMSCurrent) + 211.5;
    float MotorPowerLoss = pow(RMSCurrent, 2.0) * MotorPhaseResistance;
    float AccPowerLoss = pow(RMSCurrent, 2.0) * AccResistance;
    float motorPower = MechMotorPower - CtrlPowerLoss - MotorPowerLoss - AccPowerLoss;
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
    distanceStraight += (velocity * timeStep);
    timer += timeStep;
    StateOfCharge -= 0.0277 * timeStep;

    vel_writer<<velocity*3.6<<"km/h"<<std::endl;
    accel_writer<<acceleration<<"m/s^2"<<std::endl;
    torque_writer<<wheelTorque<<"N.m"<<std::endl;
    force_writer<<normalForceTotal<<"N"<<std::endl;
  }
  vel_writer.close();
  accel_writer.close();
  torque_writer.close();
  force_writer.close();
  return 0;
}

int TrackDrive(float &timer, float &run_distance)
{ 
  float powerPercentage = 0.5;
  RMSCurrent = RMSCurrent * powerPercentage;
  std::ofstream time_writer("/home/go/Documents/C++_Projects/lap_sim_cpp/track_run_output/setup.txt");
while(current_distance < run_distance){
  float finalDistance = 75;
  straightLine(finalDistance, timer, StateOfCharge, velocity);

  float Turn1Radius = 10.0;
  float SectorLenght = 20.0;

  Braking(Turn1Radius, SectorLenght, timer, velocity);

  Cornering(Turn1Radius, SectorLenght, timer, StateOfCharge, velocity);

  current_distance += SectorLenght + finalDistance;
}
  time_writer<<run_distance/1000.0<<"km"<<std::endl;
  time_writer<<RMSCurrent<<"A"<<std::endl;
  time_writer<<StateOfCharge<<"kWh"<<std::endl;
  time_writer<<timer<<"sec"<<std::endl;
  time_writer.close();
  return 0;
}  

int main()
{
  auto t1 = std::chrono::high_resolution_clock::now();
  int mode = 3; // 1 == accel run, 2 == SkidPad, 3 == TrackRuns
  switch(mode)
  {
    case 1:
    {
      float finalDistance = 75.0;
      straightLine(finalDistance, timer, StateOfCharge, velocity);
      break;
    }
    case 2:
    {
      //Track parameters
      float Turn1Radius = 9.125;
      float SectorLenght = 2 * M_PI * Turn1Radius;
      Cornering(Turn1Radius, SectorLenght, timer, StateOfCharge, velocity);
      break;
    }
    case 3:
    {
      float run_distance = 22000;
      TrackDrive(timer, run_distance);
      break;
    }
  }
  std::cout<<"Lap time: "<<timer<<" seconds"<<std::endl;
  std::cout<<"Lap time: "<<timer/60<<" minutes"<<std::endl;
  std::cout<<"Accumulator Capacity: "<<StateOfCharge * accVoltage<<std::endl;
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"Program Run time: "<<duration/1000<<std::endl; // milliseconds
  return 0;
}
