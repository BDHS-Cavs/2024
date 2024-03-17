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

#pragma once

#include <optional>

#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <cameraserver/CameraServer.h>

#include "Constants.h"

#include "subsystems/Drive.h"
#include "subsystems/Climber.h"
#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"
#include "subsystems/Vision.h"
#include "subsystems/Conveyer.h"

#include "commands/ClimberLowerCommand.h"
#include "commands/ClimberRaiseCommand.h"
#include "commands/ShooterShootCommand.h"
#include "commands/ShooterRetractCommand.h"
#include "commands/IntakeRunCommand.h"
#include "commands/IntakeExpelCommand.h"
#include "commands/ConveyerForwardCommand.h"
#include "commands/ConveyerBackwardCommand.h"

//#include "commands/AutonomousCommand.h"

#include "commands/LeftAuto.h"
#include "commands/RightAuto.h"
#include "commands/CenterAuto.h"

class RobotContainer {

public:



    frc2::Command* GetAutonomousCommand();
    static RobotContainer* GetInstance();

    // The robot's subsystems
    Drive m_drive;
    Climber m_climber;
    Shooter m_shooter;
    Intake m_intake;
    Vision m_vision;
    Conveyer m_conveyer;

    // Get the control values
//    frc::XboxController *getJoystick(); //drivecontroller
    frc::PS4Controller *getJoystick();
    frc::XboxController *getController();

//    frc::XboxController m_drivecontroller{0}; //drivecontroller
    frc::PS4Controller m_drivecontroller{0};  
    frc::XboxController m_controller{1};






private:

    RobotContainer();

    // Joysticks


    
    // For Multiple Autos
    // The chooser for the autonomous modes
    //frc::SendableChooser<frc2::Command*> m_chooser; //BROKEN chooser stuff
    
    // The autonomous modes //BROKEN chooser stuff
    //frc2::CommandPtr m_leftAuto = autos::LeftAuto(&m_arm, &m_drive, &m_grabber);
    //frc2::CommandPtr m_rightAuto = autos::RightAuto(&m_arm, &m_drive, &m_grabber);
    //frc2::CommandPtr m_centerAuto = autos::CenterAuto(&m_arm, &m_drive, &m_grabber);

    //frc::SendableChooser<frc2::Command*> m_chooser; //if i remove all the chooser stuff, auto crashes, so i just leave it alone but still use the single autonomouscommand.cpp

    static RobotContainer* m_robotContainer;
    void ConfigureButtonBindings();




      // The autonomous routines
  LeftAuto m_leftAuto{&m_drive, &m_climber, &m_shooter, &m_intake, &m_vision, &m_conveyer};
  RightAuto m_rightAuto{&m_drive, &m_climber, &m_shooter, &m_intake, &m_vision, &m_conveyer};
  CenterAuto m_centerAuto{&m_drive, &m_climber, &m_shooter, &m_intake, &m_vision, &m_conveyer};

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;
};