// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/ADXRS450_Gyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveModule.h"
#include "Constants.h"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
  Drivetrain() { m_gyro.Reset(); }

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::second_t period);
  void UpdateOdometry();

  static constexpr units::meters_per_second_t kMaxSpeed =
      /*3.0_mps;*/ 3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      /*std::numbers::pi*/ std::numbers::pi};  // 1/2 rotation per second

 private:
  frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
  frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
  frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
  frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

/*(int driveMotorChannel, int turningMotorChannel, 
int driveEncoderChannelA, int driveEncoderChannelB, 
int turningEncoderChannelA, int turningEncoderChannelB);*/

  SwerveModule m_frontLeft{DriveConstants::kFrontLeftDriveMotorPort, DriveConstants::kFrontLeftTurningMotorPort, DriveConstants::kFrontLeftDriveEncoderChannelA, DriveConstants::kFrontLeftDriveEncoderChannelB, DriveConstants::kFrontLeftTurningEncoderChannelA, DriveConstants::kFrontLeftTurningEncoderChannelB};
  SwerveModule m_frontRight{DriveConstants::kFrontRightDriveMotorPort, DriveConstants::kFrontRightTurningMotorPort, DriveConstants::kFrontRightDriveEncoderChannelA, DriveConstants::kFrontRightDriveEncoderChannelB, DriveConstants::kFrontRightTurningEncoderChannelA, DriveConstants::kFrontRightTurningEncoderChannelB};
  SwerveModule m_backLeft{DriveConstants::kRearLeftDriveMotorPort, DriveConstants::kRearLeftTurningMotorPort, DriveConstants::kRearLeftDriveEncoderChannelA, DriveConstants::kRearLeftDriveEncoderChannelB, DriveConstants::kRearLeftTurningEncoderChannelA, DriveConstants::kRearLeftTurningEncoderChannelB};
  SwerveModule m_backRight{DriveConstants::kRearRightDriveMotorPort, DriveConstants::kRearRightTurningMotorPort, DriveConstants::kRearRightDriveEncoderChannelA, DriveConstants::kRearRightDriveEncoderChannelB, DriveConstants::kRearRightTurningEncoderChannelA, DriveConstants::kRearRightTurningEncoderChannelB};

  frc::ADXRS450_Gyro m_gyro;

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{
      m_kinematics,
      m_gyro.GetRotation2d(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
