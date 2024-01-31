// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include "Constants.h"

//#include <ctre/Phoenix.h>

class SwerveModule {
 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel,
               const int driveEncoderPorts[2], const int turningEncoderPorts[2],
               bool driveEncoderReversed, bool turningEncoderReversed);

  frc::SwerveModuleState GetState();
  frc::SwerveModulePosition GetPosition();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr auto kModuleMaxAngularVelocity =
      units::radians_per_second_t{std::numbers::pi};
  static constexpr auto kModuleMaxAngularAcceleration =
      units::radians_per_second_squared_t{std::numbers::pi * 2.0};

    ctre::phoenix::motorcontrol::can::TalonSRX m_driveMotor;
    ctre::phoenix::motorcontrol::can::TalonSRX m_turningMotor;

    frc::Encoder m_driveEncoder;
    frc::Encoder m_turningEncoder;
    //ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice m_driveEncoder; TO USE THIS UNCOMMENT THE PHOENIX.h
    //ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice m_turningEncoder; TO USE THIS UNCOMMENT THE PHOENIX.h
    //double driveEncoderPosition = m_driveMotor.GetSelectedSensorPosition(0);
    //double driveEncoderVelocity = m_driveMotor.GetSelectedSensorVelocity(0);
    //double turningEncoderPosition = m_turningMotor.GetSelectedSensorPosition(1);
    //double turningEncoderVelocity = m_turningMotor.GetSelectedSensorVelocity(1);

  bool m_reverseDriveEncoder;
  bool m_reverseTurningEncoder;

  frc::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
};
