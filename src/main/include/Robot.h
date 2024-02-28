// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <cameraserver/CameraServer.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override;
  void TeleopPeriodic() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void TeleopInit() override;
  void TestPeriodic() override;

  void ConfigureButtonBindings();

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  //std::optional<frc2::CommandPtr> m_autonomousCommand;


       //set up default drive command real
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_container->m_drivecontroller.GetX(), 0.2)) *
                        Drivetrain::kMaxSpeed;
                        frc::SmartDashboard::PutNumber("xSpeed", xSpeed());

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_container->m_drivecontroller.GetY(), 0.2)) *
                        Drivetrain::kMaxSpeed;
                        frc::SmartDashboard::PutNumber("ySpeed", ySpeed());

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_container->m_drivecontroller.GetZ(), 0.2)) *
                     Drivetrain::kMaxAngularSpeed;
                     frc::SmartDashboard::PutNumber("zSpeed", rot());

    m_container->m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
  }

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

RobotContainer* m_container = RobotContainer::GetInstance();

};