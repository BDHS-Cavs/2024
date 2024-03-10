// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

//#include <frc/Encoder.h>
//#include <frc/controller/PIDController.h>
//#include <frc/controller/ProfiledPIDController.h>
//#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.h"

#include <units/math.h>
#include <frc/MathUtil.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorPort, int turningMotorPort);
  //frc::SwerveModuleState GetState() const;
  //frc::SwerveModulePosition GetPosition() const;
  //void SetDesiredState(const frc::SwerveModuleState& state);

  ctre::phoenix::motorcontrol::can::TalonSRX m_driveMotor;
  ctre::phoenix::motorcontrol::can::TalonSRX m_turningMotor;

  //frc::Encoder m_driveEncoder;
  //frc::Encoder m_turningEncoder;

 private:
  static constexpr double kWheelRadius = 0.0508; //need tuning?
  //TODO needed? static constexpr int kEncoderResolution = 4096; //need tuning?

  //static constexpr auto kModuleMaxAngularVelocity =
  //    std::numbers::pi * 1_rad_per_s;  // radians per second
  //static constexpr auto kModuleMaxAngularAcceleration =
  //    std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2



  //frc::PIDController m_drivePIDController{1.0, 0, 0};
  //frc::ProfiledPIDController<units::radians> m_turningPIDController{
  //    1.0,
  //    0.0,
  //    0.0,
  //    {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
//
  //frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,
  //                                                              3_V / 1_mps};
  //frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{
  //    1_V, 0.5_V / 1_rad_per_s};

};
