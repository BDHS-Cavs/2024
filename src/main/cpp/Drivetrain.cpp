// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include "frc/smartdashboard/SmartDashboard.h"

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) {
  auto states =
      m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  frc::SmartDashboard::PutNumber("Front Left Drive Encoder Distance", m_frontLeft.m_driveEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Front Right Drive Encoder Distance", m_frontRight.m_driveEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Back Left Drive Encoder Distance", m_backLeft.m_driveEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Back Right Drive Encoder Distance", m_backRight.m_driveEncoder.GetDistance());

  frc::SmartDashboard::PutNumber("Front Left Drive Encoder Get", m_frontLeft.m_driveEncoder.Get());
  frc::SmartDashboard::PutNumber("Front Right Drive Encoder Get", m_frontRight.m_driveEncoder.Get());
  frc::SmartDashboard::PutNumber("Back Left Drive Encoder Get", m_backLeft.m_driveEncoder.Get());
  frc::SmartDashboard::PutNumber("Back Right Drive Encoder Get", m_backRight.m_driveEncoder.Get());





  frc::SmartDashboard::PutNumber("Front Left Turning Encoder Distance", m_frontLeft.m_turningEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Front Right Turning Encoder Distance", m_frontRight.m_turningEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Back Left Turning Encoder Distance", m_backLeft.m_turningEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Back Right Turning Encoder Distance", m_backRight.m_turningEncoder.GetDistance());

  frc::SmartDashboard::PutNumber("Front Left Turning Encoder GetAbsolute", m_frontLeft.m_turningEncoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Front Right Turning Encoder GetAbsolute", m_frontRight.m_turningEncoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Back Left Turning Encoder GetAbsolute", m_backLeft.m_turningEncoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Back Right Turning Encoder GetAbsolute", m_backRight.m_turningEncoder.GetAbsolutePosition());

}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}
