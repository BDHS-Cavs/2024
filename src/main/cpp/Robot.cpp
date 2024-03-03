// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  frc::CameraServer::GetVideo();
  frc::CameraServer::StartAutomaticCapture();
  m_container->m_climber.ClimberIdleMode();
  m_container->m_shooter.ShooterIdleMode();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();




    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Percent)", m_container->m_swerve.m_frontLeft.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Volts)", m_container->m_swerve.m_frontLeft.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Percent)", m_container->m_swerve.m_frontLeft.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Volts)", m_container->m_swerve.m_frontLeft.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Left Drive Motor Output (Amps)", m_container->m_swerve.m_frontLeft.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Front Left Turning Motor Output (Amps)", m_container->m_swerve.m_frontLeft.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Percent)", m_container->m_swerve.m_frontRight.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Volts)", m_container->m_swerve.m_frontRight.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Percent)", m_container->m_swerve.m_frontRight.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Volts)", m_container->m_swerve.m_frontRight.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Front Right Drive Motor Output (Amps)", m_container->m_swerve.m_frontRight.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Front Right Turning Motor Output (Amps)", m_container->m_swerve.m_frontRight.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Percent)", m_container->m_swerve.m_backLeft.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Volts)", m_container->m_swerve.m_backLeft.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Percent)", m_container->m_swerve.m_backLeft.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Volts)", m_container->m_swerve.m_backLeft.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Left Drive Motor Output (Amps)", m_container->m_swerve.m_backLeft.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Back Left Turning Motor Output (Amps)", m_container->m_swerve.m_backLeft.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Percent)", m_container->m_swerve.m_backRight.m_driveMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Volts)", m_container->m_swerve.m_backRight.m_driveMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Percent)", m_container->m_swerve.m_backRight.m_turningMotor.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Volts)", m_container->m_swerve.m_backRight.m_turningMotor.GetMotorOutputVoltage());
    frc::SmartDashboard::PutNumber("Back Right Drive Motor Output (Amps)", m_container->m_swerve.m_backRight.m_driveMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Back Right Turning Motor Output (Amps)", m_container->m_swerve.m_backRight.m_turningMotor.GetOutputCurrent());

    frc::SmartDashboard::PutNumber("Back Right Turning Encoder", m_container->m_swerve.m_backRight.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Back Right Drive Encoder", m_container->m_swerve.m_backRight.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Right Turning Encoder", m_container->m_swerve.m_frontRight.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Right Drive Encoder", m_container->m_swerve.m_frontRight.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Back Left Turning Encoder", m_container->m_swerve.m_backLeft.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Back Left Drive Encoder", m_container->m_swerve.m_backLeft.m_driveEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Turning Encoder", m_container->m_swerve.m_frontLeft.m_turningEncoder.GetDistance());
    frc::SmartDashboard::PutNumber("Front Left Drive Encoder", m_container->m_swerve.m_frontLeft.m_driveEncoder.GetDistance());





}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container->GetAutonomousCommand();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
//TODO CRASHES AUTO? //  DriveWithJoystick(false);
//TODO CRASHES AUTO? //m_container->m_swerve.UpdateOdometry();
m_autonomousCommand->Execute();
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  DriveWithJoystick(true);
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif