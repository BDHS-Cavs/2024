// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  ConfigureButtonBindings();
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
  //m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  DriveWithJoystick(false);
m_swerve.UpdateOdometry();
}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();
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


void Robot::ConfigureButtonBindings() {
//frc2::JoystickButton m_controllerButton1{&m_controller, (int)frc::XboxController::Button::kA};           // ? (1)
frc2::JoystickButton m_controllerButton2{&m_controller, (int)frc::XboxController::Button::kB};           // Climber Raise (2)
//frc2::JoystickButton m_controllerButton3{&m_controller, (int)frc::XboxController::Button::kX};           // Arm Extend (3)
frc2::JoystickButton m_controllerButton4{&m_controller, (int)frc::XboxController::Button::kY};           // Shooter Shoot (4)
//frc2::JoystickButton m_controllerButton5{&m_controller, (int)frc::XboxController::Button::kLeftBumper};  // Grabber Open (5)
//frc2::JoystickButton m_controllerButton6{&m_controller, (int)frc::XboxController::Button::kRightBumper}; // grabber Close (6)

//frc2::JoystickButton m_controllerButton7{&m_controller, (int)frc::XboxController::Button::kBack};        // Compressor Enable (7)
//frc2::JoystickButton m_controllerButton8{&m_controller, (int)frc::XboxController::Button::kStart};       // Cam Backwards (8)

m_controllerButton2.WhileTrue(ClimberRaiseCommand(&m_climber).ToPtr());               // Climber Raise (2)
m_controllerButton4.WhileTrue(ShooterShootCommand(&m_shooter).ToPtr());              // Shooter Shoot (4)
//m_controllerButton1.WhileTrue(ArmRetractCommand(&m_arm).ToPtr());             // Arm Retract (1)
//m_controllerButton3.WhileTrue(ArmExtendCommand(&m_arm).ToPtr());              // Arm Extend (3)
//m_controllerButton5.OnTrue(GrabberOpenCommand(&m_grabber).ToPtr());           // Grabber Open (5)
//m_controllerButton6.OnTrue(GrabberCloseCommand(&m_grabber).ToPtr());            // Grabber Close (6)

//m_controllerButton7.ToggleOnTrue(CompressorEnableCommand(&m_grabber).ToPtr()); // Compressor Enable (7)
//m_controllerButton8.OnTrue(CamBackwardsCommand(&m_drive).ToPtr());       // Cam Backwards (8)
}




#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif