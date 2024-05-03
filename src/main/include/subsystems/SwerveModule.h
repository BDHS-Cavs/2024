#pragma once

#include <numbers>

#include "Constants.h"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "frc/AnalogEncoder.h"
#include "frc/Encoder.h"

class SwerveModule {
 public:
  SwerveModule(int turningMotorPort, int driveMotorPort, int turningAnalogEncoderPort, int driveEncoderPorts[2]);
  void Drive(double speed, double angle);
  int getAnglePosition();
  int getDrivePosition();

  ctre::phoenix::motorcontrol::can::TalonSRX m_driveMotor;
  ctre::phoenix::motorcontrol::can::TalonSRX m_turningMotor;
  frc::AnalogEncoder m_turningAnalogEncoder;
  frc::Encoder m_driveEncoder;
};