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

    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Percent)", m_backLeft.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Volts)", m_backLeft.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Percent)", m_backLeft.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Volts)", m_backLeft.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Amps)", m_backLeft.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Amps)", m_backLeft.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Percent)", m_backRight.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Volts)", m_backRight.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Percent)", m_backRight.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Volts)", m_backRight.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Amps)", m_backRight.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Amps)", m_backRight.m_turningMotor.GetOutputCurrent());

        frc::SmartDashboard::PutNumber("Back Right Turning Encoder", m_backRight.m_turningEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Back Right Drive Encoder", m_backRight.m_driveEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Front Right Turning Encoder", m_frontRight.m_turningEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Front Right Drive Encoder", m_frontRight.m_driveEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Back Left Turning Encoder", m_backLeft.m_turningEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Back Left Drive Encoder", m_backLeft.m_driveEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Front Left Turning Encoder", m_frontLeft.m_turningEncoder.GetDistance());
        frc::SmartDashboard::PutNumber("Front Left Drive Encoder", m_frontLeft.m_driveEncoder.GetDistance());



    //frc::SmartDashboard::PutNumber("Front Left Drive Encoder Rate", m_frontLeft.m_driveEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Front Left Turning Encoder Rate", m_frontLeft.m_turningEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Front Left Drive Encoder Current Count", m_frontLeft.m_driveEncoder.Get());
    //frc::SmartDashboard::PutNumber("Front Left Turning Encoder Current Count", m_frontLeft.m_turningEncoder.Get());

    //frc::SmartDashboard::PutNumber("Front Right Drive Encoder Rate", m_frontRight.m_driveEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Front Right Turning Encoder Rate", m_frontRight.m_turningEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Front Right Drive Encoder Current Count", m_frontRight.m_driveEncoder.Get());
    //frc::SmartDashboard::PutNumber("Front Right Turning Encoder Current Count", m_frontRight.m_turningEncoder.Get());

    //frc::SmartDashboard::PutNumber("Back Left Drive Encoder Rate", m_backLeft.m_driveEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Back Left Turning Encoder Rate", m_backLeft.m_turningEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Back Left Drive Encoder Current Count", m_backLeft.m_driveEncoder.Get());
    //frc::SmartDashboard::PutNumber("Back Left Turning Encoder Current Count", m_backLeft.m_turningEncoder.Get());

    //frc::SmartDashboard::PutNumber("Back Right Drive Encoder Rate", m_backRight.m_driveEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Back Right Turning Encoder Rate", m_backRight.m_turningEncoder.GetRate());
    //frc::SmartDashboard::PutNumber("Back Right Drive Encoder Current Count", m_backRight.m_driveEncoder.Get());
    //frc::SmartDashboard::PutNumber("Back Right Turning Encoder Current Count", m_backRight.m_turningEncoder.Get());

    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_frontLeft.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_frontRight.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_backLeft.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_backRight.m_driveEncoder.GetDistance());

    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_frontLeft.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_frontRight.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_backLeft.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder GetDistance", m_backRight.m_turningEncoder.GetDistance());

    frc::SmartDashboard::PutNumber("gyro get angle", m_gyro.GetAngle());
    frc::SmartDashboard::PutNumber("gyro get rate", m_gyro.GetRate());




}

void Drivetrain::UpdateOdometry() {
  m_odometry.Update(m_gyro.GetRotation2d(),
                    {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                     m_backLeft.GetPosition(), m_backRight.GetPosition()});
}
