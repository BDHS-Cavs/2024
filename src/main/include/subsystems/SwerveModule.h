#pragma once

#include <numbers>

#include "Constants.h"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

class SwerveModule {
 public:
  SwerveModule(int turningMotorPort, int driveMotorPort);
  void Drive(double speed, double angle);
  int getAnglePosition();
  int getDrivePosition();

  ctre::phoenix::motorcontrol::can::TalonSRX m_driveMotor;
  ctre::phoenix::motorcontrol::can::TalonSRX m_turningMotor;

};