// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

void Drivetrain::Drive(double speed, double angle) {
  //TODO needed? auto states =
  //TODO needed?     m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
  //TODO needed?         fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
  //TODO needed?                             xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
  //TODO needed?                       : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
  //TODO needed?         period));

  //TODO needed? m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  //TODO needed? auto [fl, fr, bl, br] = states;

  //TODO needed? m_frontLeft.SetDesiredState(fl);
  //TODO needed? m_frontRight.SetDesiredState(fr);
  //TODO needed? m_backLeft.SetDesiredState(bl);
  //TODO needed? m_backRight.SetDesiredState(br);

 //m_frontLeft.m_driveEncoder.SetMinRate(3);
 //m_frontRight.m_driveEncoder.SetMinRate(3);
 //m_backLeft.m_driveEncoder.SetMinRate(3);
 //m_backRight.m_driveEncoder.SetMinRate(3);
 //
 //m_frontLeft.m_turningEncoder.SetMinRate(3);
 //m_frontRight.m_turningEncoder.SetMinRate(3);
 //m_backLeft.m_turningEncoder.SetMinRate(3);
 //m_backRight.m_turningEncoder.SetMinRate(3);




}

//TODO needed? void Drivetrain::UpdateOdometry() {
//TODO needed?   m_odometry.Update(m_gyro.GetRotation2d(),
//TODO needed?                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
//TODO needed?                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
//TODO needed? }

void Drivetrain::DrivetrainStop() {
    m_frontLeft.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_frontRight.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_backLeft.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_backRight.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

    m_frontLeft.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_frontRight.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_backLeft.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_backRight.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}