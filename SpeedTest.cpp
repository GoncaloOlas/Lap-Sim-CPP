#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <algorithm>
#include <windows.h>
#include <thread>
#include <fstream>

int main()
{
  auto t1 = std::chrono::high_resolution_clock::now();
  //Simulation parameters
  float timeStep = 0.001; //seconds
  float finalDistance = 75.0; //meters
  float time = 0.0;

  //enviromental parameters
  float gravity = 9.81;
  float rho = 1.162;

  //vehicle parameters
  float mass = 230.0 + 68.0; //Car + driver
  float frictionCoef = 1.0;
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

  float motorTorque = 0.0;
  float rpmTire = 0.0;
  float velocity = 0.0;
  float distance = 0.0;

  std::ofstream writer("Output\\Velocity.txt");
  std::ofstream writer1("Output\\Force.txt");
  std::ofstream writer2("Output\\RPM.txt");
  std::ofstream writer3("Output\\Torque.txt");
  std::ofstream writer4("Output\\time.txt");
  std::ofstream writer5("Output\\distance.txt");
  std::ofstream writer6("Output\\Acceleration.txt");


  while(finalDistance >= distance)
  {
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
    float downForce = 0.5 * rho * frontalArea * liftCoef * pow(velocity, 2.0);
    float dragForce = 0.5 * rho * frontalArea * dragCoef * pow(velocity, 2.0);
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
    time += timeStep;


    std::cout<<"Real force= "<<realForce<<std::endl;
    std::cout<<"Mass= "<<mass<<std::endl;
    std::cout<<"Accel = "<<acceleration<<std::endl;
    std::cout<<"\n\n"<<std::endl;


    writer<<velocity*3.6<<std::endl;
    writer1<<realForce<<std::endl;
    writer2<<rpmTire<<std::endl;
    writer3<<motorTorque<<std::endl;
    writer4<<time<<std::endl;
    writer5<<distance<<std::endl;
    writer6<<acceleration<<std::endl;
  }
  writer<<time<<std::endl;
  std::cout<<"Accel time: "<<time<<std::endl;
  writer.close();
  writer1.close();
  writer2.close();
  writer3.close();
  writer4.close();
  writer5.close();
  writer6.close();
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout<<"Program Run time: "<<duration/1000<<std::endl; //milliseconds
  return 0;
}
