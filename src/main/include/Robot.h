// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Climber.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

#include "commands/ClimberLowerCommand.h"
#include "commands/ClimberRaiseCommand.h"
#include "commands/ShooterShootCommand.h"
#include "commands/IntakeRunCommand.h"
#include "commands/IntakeExpelCommand.h"

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
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  frc::Joystick m_drivecontroller{0};
  frc::XboxController m_controller{1};
  
  //the robot subsystems
  Drivetrain m_swerve;
  Climber m_climber;
  Shooter m_shooter;
  Intake m_intake;


  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};

  void DriveWithJoystick(bool fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed = -m_xspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_drivecontroller.GetX(), 0.2)) *
                        Drivetrain::kMaxSpeed;
                        frc::SmartDashboard::PutNumber("xSpeed", xSpeed());

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed = -m_yspeedLimiter.Calculate(
                            frc::ApplyDeadband(m_drivecontroller.GetY(), 0.2)) *
                        Drivetrain::kMaxSpeed;
                        frc::SmartDashboard::PutNumber("ySpeed", ySpeed());

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_rotLimiter.Calculate(
                         frc::ApplyDeadband(m_drivecontroller.GetZ(), 0.2)) *
                     Drivetrain::kMaxAngularSpeed;
                     frc::SmartDashboard::PutNumber("zSpeed", rot());

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, GetPeriod());
  }


};