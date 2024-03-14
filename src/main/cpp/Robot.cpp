// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  frc::CameraServer::GetVideo(); //i dont know if we need this line because it says use this if you want to use vision processing on the roborio but we do it on the 2nd camera (the limelight 2+) with photonvision
  frc::CameraServer::StartAutomaticCapture(); //make usb cam (microsoft lifecam hd-3000) work

  m_container->m_drive.m_drivegyro.Calibrate(); //calibrate gyro
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

    //drive
          //driving motors
    frc::SmartDashboard::PutNumber("Front Left Drive Percent Output", m_container->m_drive.m_leftFront.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Drive Percent Output", m_container->m_drive.m_leftRear.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Drive Percent Output", m_container->m_drive.m_rightFront.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Drive Percent Output", m_container->m_drive.m_rightRear.GetMotorOutputPercent());
          //turning motors
    frc::SmartDashboard::PutNumber("Front Left Turning Percent Output", m_container->m_drive.m_turningLeftFront.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Left Turning Percent Output", m_container->m_drive.m_turningLeftRear.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Front Right Turning Percent Output", m_container->m_drive.m_turningRightFront.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("Back Right Turning Percent Output", m_container->m_drive.m_turningRightRear.GetMotorOutputPercent());

    //gyro
    frc::SmartDashboard::PutNumber("gyro get angle", m_container->m_drive.m_drivegyro.GetAngle());
    frc::SmartDashboard::PutNumber("gyro get rate", m_container->m_drive.m_drivegyro.GetRate());

    //getperiod
    frc::SmartDashboard::PutNumber("Swerve GetPeriod", GetPeriod().value());
    //frc::SmartDashboard::PutNumber("Gyro Rotation2d", m_container->m_swerve.m_gyro.GetRotation2d()); //BROKEN

  //shooter
    frc::SmartDashboard::PutNumber("Shooter Encoder 1 Position", m_container->m_shooter.m_shooterEncoder1.GetPosition());
    frc::SmartDashboard::PutNumber("Shooter Encoder 2 Position", m_container->m_shooter.m_shooterEncoder2.GetPosition());

    frc::SmartDashboard::PutNumber("Shooter Encoder 1 Velocity", m_container->m_shooter.m_shooterEncoder1.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Encoder 2 Velocity", m_container->m_shooter.m_shooterEncoder2.GetVelocity());

  //climber
    frc::SmartDashboard::PutNumber("Climber Encoder 1 Position", m_container->m_climber.m_climberEncoder1.GetPosition());
    frc::SmartDashboard::PutNumber("Climber Encoder 2 Position", m_container->m_climber.m_climberEncoder2.GetPosition());

    frc::SmartDashboard::PutNumber("Climber Encoder 1 Velocity", m_container->m_shooter.m_shooterEncoder1.GetVelocity());
    frc::SmartDashboard::PutNumber("Climber Encoder 2 Velocity", m_container->m_shooter.m_shooterEncoder2.GetVelocity());

  //vision
    //TODO remove? m_container->m_vision.VisionTrack();
    //TODO remove? frc::SmartDashboard::PutNumber("Photon current ID", m_container->m_vision.target.GetFiducialId());
    //TODO remove? frc::SmartDashboard::PutBoolean("Photon HasTargets", m_container->m_vision.result.HasTargets());
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
//TODO CRASHES AUTO? but i dont think we need bc joystick disabled in auto anyway //  DriveWithJoystick(false);
//TODO remove? remove the whole function? m_container->m_swerve.UpdateOdometry();
//m_autonomousCommand->Execute();
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  //TODO: might want to swerve update odometry and might want to reset climber/shooter encoders
  
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  //DriveWithJoystick(true);

//swerve drive with joystick
//m_container->m_swerve.calculateVectors(m_container->getX(), m_container->getY(), m_container->getZ());
//m_container->m_swerve.calculateVectors(1, 1, 0);

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