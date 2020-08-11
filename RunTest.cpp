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

//enviromental parameters
float gravity = 9.81;
float rho = 1.162;

//Track parameters
float segment1 = 500.0;
float segment2 = 500.0;
float turn1Rad = 50.0;
float turn2Rad = 100.0;
float perimeter1 = 2 * M_PI *turn1Rad;
float perimeter2 = 2 * M_PI *turn2Rad;
float dd = 1.0;

//vehicle parameters
float mass = 230.0 + 68.0; //Car + driver
float frictionCoef = 1.4;
float muLong = 1.4;
float dragCoef = 0.5;
float liftCoef = 0.12;
float frontalArea = 1.02;
float tireRadius = 0.2286;
float finalDriveRatio = 3.155;
float accVoltage = 399.0;
float rpmCoef = 470.0 / 5170.0;
float motorPower = 80000.0;
float maxRpmMotor = accVoltage / rpmCoef;
float tirePerimeter = 2 * M_PI * tireRadius;
float CG_long = 0.49;  // percent rear
float CP_long = 0.54;  // percent rear
float CG_vert = 0.3124;

float motorTorque = 0.0;
float rpmTire = 0.0;
float velocity = 0.0;
float distance = 0.0;

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

    float normalFrontForce = (mass * gravity * (1 - CG_long)) + (dragForce * (1 - CP_long));
    float normalRearForce = (mass * gravity * CG_long) + (dragForce * CP_long);
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
/*
int Cornering(float segment1, float segment2, float turn1Rad, float turn2Rad){
std::ofstream writer("OutputRun\\Velocity.txt");


float perimeter1 = 2 * M_PI * turn1Rad;
float perimeter2 = 2 * M_PI * turn2Rad;
velocity = sqrt(GtoGRadius * gravity * turn1Rad);
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
float force = wheelTorque / tireRadius;
float normalForce = mass * gravity + downForce;
float frictionForce = frictionCoef * normalForce;
float motorExactPower = motorPower / velocity;
float motorForce = std::min(force, motorExactPower);
float realForce = std::min(frictionForce, motorForce) - dragForce;
float acceleration = realForce / mass;

velocity

distance += (velocity * timeStep);
timer += timeStep;
std::cout<<distance<<std::endl;
/*
float Corner1Force = (mass * (velocity * velocity)) / turn1Rad;
float Corner2Force = (mass * (velocity * velocity)) / turn2Rad;

float acceleration1 = Corner1Force / mass;
//float acceleration2 = Corner2Force / mass;

velocity = (acceleration1 * timeStep);
distance += (velocity * timeStep);
timer += timeStep;
std::cout<<velocity<<std::endl;
writer<<velocity<<std::endl;
//std::cout<<acceleration1<<std::endl;

}


//float VelApex1 = sqrt(GtoGRadius * gravity * turn1Rad);
//float VelApex2 = sqrt(GtoGRadius * gravity * turn2Rad);

writer.close();
std::cout<<"Cornering"<<std::endl;
return timer;
}
*/
int straightLine(float finalDistance)
{
  while(finalDistance >= distance)
  {
    if(rpmTire < maxRpmMotor)
    {
      rpmTire = (velocity /tirePerimeter) * 60 * finalDriveRatio;
    }else
    {
      rpmTire = maxRpmMotor;
    }
    motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    if(motorTorque > 240)
    {
      motorTorque = 240;
    }else
    {
      motorTorque = motorPower / ((rpmTire * 2 * M_PI) / 60);
    }
    float downForce = 0.5 * rho * frontalArea * liftCoef * (velocity*velocity);
    float dragForce = 0.5 * rho * frontalArea * dragCoef * (velocity*velocity);
    float wheelTorque = motorTorque * finalDriveRatio;
    float force = wheelTorque / tireRadius;
    float normalForce = mass * gravity + downForce;
    float frictionForce = frictionCoef * normalForce;
    float motorExactPower = motorPower / velocity;
    float motorForce = std::min(force, motorExactPower);
    float realForce = std::min(frictionForce, motorForce) - dragForce;
    float acceleration = realForce / mass;

    velocity += (acceleration * timeStep);
    distance += (velocity * timeStep);
    timer += timeStep;
  }
  return timer;
}
int main()
{
  auto t1 = std::chrono::high_resolution_clock::now();
  int mode = 2; // 1 == accel, 2 == track
  switch(mode)
  {
    case 1:
    {
      float finalDistance = 75.0;
      straightLine(finalDistance);
      break;
    }
    case 2:
    {
      //Track parameters
      float segment1 = 500.0;
      float segment2 = 500.0;
      float turn1Rad = 9.125;
      float turn2Rad = 9.125;

      Cornering(segment1, segment2, turn1Rad, turn2Rad);
      break;
    }
  }
  std::cout<<"Accel time: "<<timer<<std::endl;
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"Program Run time: "<<duration/1000<<std::endl; // milliseconds
  return 0;
}
