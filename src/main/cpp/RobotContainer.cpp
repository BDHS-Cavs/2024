// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

#include "RobotContainer.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer* RobotContainer::m_robotContainer = NULL;

RobotContainer::RobotContainer() 
: fl(DriveConstants::kFrontLeftDriveMotorPort, DriveConstants::kFrontLeftTurningMotorPort),
  fr(DriveConstants::kFrontRightDriveMotorPort, DriveConstants::kFrontRightTurningMotorPort),
  bl(DriveConstants::kRearLeftDriveMotorPort, DriveConstants::kRearLeftTurningMotorPort),
  br(DriveConstants::kRearRightDriveMotorPort, DriveConstants::kRearRightTurningMotorPort),
  m_swerve(fl, fr, bl, br)
{
//RobotContainer::RobotContainer() : m_autonomousCommand(&m_swerve, &m_climber, &m_shooter, &m_intake, &m_vision) { //broken??
    //frc::SmartDashboard::PutData(&m_swerve); //broken but i dont think we need
    frc::SmartDashboard::PutData(&m_climber);
    frc::SmartDashboard::PutData(&m_shooter);
    frc::SmartDashboard::PutData(&m_intake);
    frc::SmartDashboard::PutData(&m_vision);
    frc::SmartDashboard::PutData(&m_conveyer);

    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("Autonomous Command", new AutonomousCommand(&m_swerve, &m_climber, &m_shooter, &m_intake, &m_vision, &m_conveyer));
	
    ConfigureButtonBindings();

    m_chooser.SetDefaultOption("Autonomous Command", new AutonomousCommand(&m_swerve, &m_climber, &m_shooter, &m_intake, &m_vision, &m_conveyer));
    frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
}

RobotContainer* RobotContainer::GetInstance() {
    if (m_robotContainer == NULL) {
        m_robotContainer = new RobotContainer();
    }
    return(m_robotContainer);
}

void RobotContainer::ConfigureButtonBindings() {                                                           // MAKE SURE YOU ARE ON THE "X" NOT "D" ON BACK OF CONTROLLER
frc2::JoystickButton m_controllerButton1{&m_controller, (int)frc::XboxController::Button::kA};             // Shooter Retract (1)   (A)
frc2::JoystickButton m_controllerButton2{&m_controller, (int)frc::XboxController::Button::kB};             // Climber Raise (2)     (B)
frc2::JoystickButton m_controllerButton3{&m_controller, (int)frc::XboxController::Button::kX};             // Climber Lower (3)     (X)
frc2::JoystickButton m_controllerButton4{&m_controller, (int)frc::XboxController::Button::kY};             // Shooter Shoot (4)     (Y)
frc2::JoystickButton m_controllerButton5{&m_controller, (int)frc::XboxController::Button::kLeftBumper};    // Intake Expel (5)      (LB)
frc2::JoystickButton m_controllerButton6{&m_controller, (int)frc::XboxController::Button::kRightBumper};   // Intake Run (6)        (RB)
frc2::JoystickButton m_controllerButton7{&m_controller, (int)frc::XboxController::Button::kBack};          // Conveyer Forward (7)  (back)
frc2::JoystickButton m_controllerButton8{&m_controller, (int)frc::XboxController::Button::kStart};         // Conveyer Backward (8) (start)

m_controllerButton1.WhileTrue(ShooterRetractCommand(&m_shooter).ToPtr());                                  // Shooter Retract (1)   (A)
m_controllerButton2.WhileTrue(ClimberRaiseCommand(&m_climber).ToPtr());                                    // Climber Raise (2)     (B)
m_controllerButton3.WhileTrue(ClimberLowerCommand(&m_climber).ToPtr());                                    // Climber Lower (3)     (X)
m_controllerButton4.WhileTrue(ShooterShootCommand(&m_shooter).ToPtr());                                    // Shooter Shoot (4)     (Y)
m_controllerButton5.WhileTrue(IntakeExpelCommand(&m_intake).ToPtr());                                        // Intake Expel (5)      (LB)
m_controllerButton6.WhileTrue(IntakeRunCommand(&m_intake).ToPtr());                                      // Intake Run (6)        (RB)
m_controllerButton7.WhileTrue(ConveyerForwardCommand(&m_conveyer).ToPtr());                                // Conveyer Forward (7)  (back)
m_controllerButton8.WhileTrue(ConveyerBackwardCommand(&m_conveyer).ToPtr());                               // Conveyer Backward (8) (start)
}

frc::Joystick* RobotContainer::getJoystick() {
   return &m_drivecontroller;
}
frc::XboxController* RobotContainer::getController() {
   return &m_controller;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // The selected command will be run in autonomous
  return m_chooser.GetSelected();
}



 double RobotContainer::getX() {
         return abs(m_drivecontroller.GetX()) > DriveConstants::DEADZONE_XY ? m_drivecontroller.GetX() : 0;
     }
 double RobotContainer::getY() {
         return abs(m_drivecontroller.GetY()) > DriveConstants::DEADZONE_XY ? m_drivecontroller.GetY() : 0;
     }
 double RobotContainer::getZ() {
         double z = m_drivecontroller.GetZ();
         // double adjustedZ = RobotMap.SENSITIVITY_Z % 2 == 0 ? Math.signum(z) * Math.pow(z,
         // RobotMap.SENSITIVITY_Z);
         return abs(z) > DriveConstants::DEADZONE_Z
                 ? (z * pow(z, DriveConstants::SENSITIVITY_Z)) / DriveConstants::REDUCER_Z
                 : 0;
     }