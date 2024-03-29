// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort,
                  kFrontLeftTurningMotorPort,
                  kFrontLeftDriveEncoderPorts,
                  kFrontLeftTurningEncoderPorts,
                  kFrontLeftDriveEncoderReversed,
                  kFrontLeftTurningEncoderReversed},

      m_rearLeft{
          kRearLeftDriveMotorPort,       kRearLeftTurningMotorPort,
          kRearLeftDriveEncoderPorts,    kRearLeftTurningEncoderPorts,
          kRearLeftDriveEncoderReversed, kRearLeftTurningEncoderReversed},

      m_frontRight{
          kFrontRightDriveMotorPort,       kFrontRightTurningMotorPort,
          kFrontRightDriveEncoderPorts,    kFrontRightTurningEncoderPorts,
          kFrontRightDriveEncoderReversed, kFrontRightTurningEncoderReversed},

      m_rearRight{
          kRearRightDriveMotorPort,       kRearRightTurningMotorPort,
          kRearRightDriveEncoderPorts,    kRearRightTurningEncoderPorts,
          kRearRightDriveEncoderReversed, kRearRightTurningEncoderReversed},

      m_odometry{kDriveKinematics,
                 m_gyro.GetRotation2d(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {}

void DriveSubsystem::Periodic() {
    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Percent)", m_frontLeft.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Volts)", m_frontLeft.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Percent)", m_frontLeft.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Volts)", m_frontLeft.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Amps)", m_frontLeft.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Amps)", m_frontLeft.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Percent)", m_frontRight.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Volts)", m_frontRight.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Percent)", m_frontRight.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Volts)", m_frontRight.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Amps)", m_frontRight.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Amps)", m_frontRight.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Rear Left Drive Motor Output (Percent)", m_rearLeft.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Rear Left Drive Motor Output (Volts)", m_rearLeft.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Rear Left Turning Motor Output (Percent)", m_rearLeft.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Rear Left Turning Motor Output (Volts)", m_rearLeft.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Rear Left Drive Motor Output (Amps)", m_rearLeft.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Rear Left Turning Motor Output (Amps)", m_rearLeft.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Rear Right Drive Motor Output (Percent)", m_rearRight.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Rear Right Drive Motor Output (Volts)", m_rearRight.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Rear Right Turning Motor Output (Percent)", m_rearRight.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Rear Right Turning Motor Output (Volts)", m_rearRight.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Rear Right Drive Motor Output (Amps)", m_rearRight.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Rear Right Turning Motor Output (Amps)", m_rearRight.m_turningMotor.GetOutputCurrent());



    frc::SmartDashboard::PutNumber("Front Left Drive Encoder Rate", m_frontLeft.m_driveEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Front Left Turning Encoder Rate", m_frontLeft.m_turningEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder Current Count", m_frontLeft.m_driveEncoder.Get());
    frc::SmartDashboard::PutNumber("Front Left Turning Encoder Current Count", m_frontLeft.m_turningEncoder.Get());

    frc::SmartDashboard::PutNumber("Front Right Drive Encoder Rate", m_frontRight.m_driveEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Front Right Turning Encoder Rate", m_frontRight.m_turningEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Front Right Drive Encoder Current Count", m_frontRight.m_driveEncoder.Get());
    frc::SmartDashboard::PutNumber("Front Right Turning Encoder Current Count", m_frontRight.m_turningEncoder.Get());

    frc::SmartDashboard::PutNumber("Rear Left Drive Encoder Rate", m_rearLeft.m_driveEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Rear Left Turning Encoder Rate", m_rearLeft.m_turningEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Rear Left Drive Encoder Current Count", m_rearLeft.m_driveEncoder.Get());
    frc::SmartDashboard::PutNumber("Rear Left Turning Encoder Current Count", m_rearLeft.m_turningEncoder.Get());

    frc::SmartDashboard::PutNumber("Rear Right Drive Encoder Rate", m_rearRight.m_driveEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Rear Right Turning Encoder Rate", m_rearRight.m_turningEncoder.GetRate());
    frc::SmartDashboard::PutNumber("Rear Right Drive Encoder Current Count", m_rearRight.m_driveEncoder.Get());
    frc::SmartDashboard::PutNumber("Rear Right Turning Encoder Current Count", m_rearRight.m_turningEncoder.Get());

  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetRotation2d().Degrees();
}

void DriveSubsystem::ZeroHeading() {
  m_gyro.Reset();
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
