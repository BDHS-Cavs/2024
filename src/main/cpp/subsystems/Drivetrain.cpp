// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

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

 //m_frontLeft.m_driveEncoder.SetMinRate(3);
 //m_frontRight.m_driveEncoder.SetMinRate(3);
 //m_backLeft.m_driveEncoder.SetMinRate(3);
 //m_backRight.m_driveEncoder.SetMinRate(3);
 //
 //m_frontLeft.m_turningEncoder.SetMinRate(3);
 //m_frontRight.m_turningEncoder.SetMinRate(3);
 //m_backLeft.m_turningEncoder.SetMinRate(3);
 //m_backRight.m_turningEncoder.SetMinRate(3);




    frc::SmartDashboard::PutNumber("gyro get angle", m_gyro.GetAngle());
    frc::SmartDashboard::PutNumber("gyro get rate", m_gyro.GetRate());




}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}
