#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

constexpr float _USE_MATH_DEFINES = 3.14159265358979323846;

constexpr float gravity = 9.81;
constexpr float rho = 1.162;

constexpr float mass = 230.0 + 68.0; // Car + driver
constexpr float frictionCoef = 1.0;
constexpr float muLong = 1.4;
constexpr float dragCoef = 0.5;
constexpr float liftCoef = 0.12;
constexpr float frontalArea = 1.02;
constexpr float tireRadius = 0.2286;
constexpr float finalDriveRatio = 3.38;
constexpr float accVoltage = 399.0;
constexpr float rpmCoef = 470.0 / 5170.0;
constexpr float maxRpmMotor = accVoltage / rpmCoef;
constexpr float tirePerimeter = 2 * _USE_MATH_DEFINES * tireRadius;
constexpr float CG_long = 0.45;  // Rear Weight distribution
constexpr float CP_long = 0.54;  // percent rear
constexpr float CG_vert = 0.300;
constexpr float WheelBase = 1.56;
constexpr float FrontTrack = 1.180;
constexpr float RearTrack = 1.160;
constexpr float RollingDrag = 100.0; // For accuracy should be changed to 200 + (250 * pow(LateralGs, 2.0))
constexpr float RMSCurrent = 172.5;
constexpr float PFactor = 0.94;
constexpr float MotorPhaseResistance = 0.007;
constexpr float MaxCurrent = 350.0;
constexpr float cellResistance = 0.003;
constexpr float AccCellNr = 95.0;
constexpr float AccResistance = cellResistance * AccCellNr;
constexpr float DrivetrainEffic = 0.85;
constexpr float BatteryCap = 20.0; // Ah
constexpr float WingArea = 0.0;
constexpr float WingDragCoef = 0.0;
constexpr float timeStep = 0.001; // seconds

#endif // CONSTANTS_HPP