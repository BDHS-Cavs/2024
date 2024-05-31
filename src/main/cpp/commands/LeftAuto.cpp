// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.
#include "commands/LeftAuto.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h> // for wpi outs()

LeftAuto::LeftAuto(Drive* m_drive, Climber* m_climber, Shooter* m_shooter, Intake* m_intake, Vision* m_vision, Conveyer* m_conveyer)
:m_drive(m_drive),
m_climber(m_climber),
m_shooter(m_shooter),
m_intake(m_intake),
m_vision(m_vision),
m_conveyer(m_conveyer)
{

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("LeftAuto");
    AddRequirements(m_drive);
    AddRequirements(m_climber);
    AddRequirements(m_shooter);
    AddRequirements(m_intake);
    AddRequirements(m_vision);
    AddRequirements(m_conveyer);

    m_firstTime = true;
    m_timer.Reset();
}

// Called just before this Command runs the first time
void LeftAuto::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void LeftAuto::Execute() {

    units::second_t period1 = 1_s; //starts at 1 (2s)
    units::second_t period2 = 3_s; //starts at 3 (3s)
    units::second_t period3 = 5_s; //starts at 5 (2s)
    units::second_t period4 = 5.5_s; //starts at 5.5 (0.5s)
    units::second_t period5 = 6.25_s; //starts at 6.25 (0.75s)
    units::second_t period6 = 7.55_s; //starts at 7.55 (1.3s)

    if(m_firstTime)
    {
        m_timer.Reset();
        m_firstTime = false;
    }

    m_timer.Start();

    frc::SmartDashboard::PutNumber("Autonomous Command Timer", double(m_timer.Get()));

    if (m_timer.Get() >= units::second_t(0_s) && m_timer.Get() < period1)
    {
        //do nothing
    }
    else if(m_timer.Get() >= period1 && m_timer.Get() < period2) //starts at 1 ends at 3 (2s)
    {
        m_shooter->ShooterShoot(); //start running the shooter
    }
    else if(m_timer.Get() >= period2 && m_timer.Get() < period3) //starts at 3 ends at 5 (2s)
    {
        m_conveyer->ConveyerForward(); //start running the conveyer
    }
    else if(m_timer.Get() >= period3 && m_timer.Get() < period4) //starts at 5 ends at 5.5 (0.5s)
    {
        m_shooter->ShooterStop(); //stop running the shooter
        m_conveyer->ConveyerStop(); //stop running the conveyer
        m_drive->AutoMotivateBackward(); //start driving backwards
    }
    else if(m_timer.Get() >= period4 && m_timer.Get() < period5) //starts at 5.5 ends at 6.25 (0.75s)
    {
        m_drive->AutoMotivateRotateLeft(); //start rotating to the left
    }
    else if(m_timer.Get() >= period5 && m_timer.Get() < period6) //starts at 6.25 ends at 7.55 (1.3s)
    {
        m_drive->AutoMotivateBackward(); //start driving backwards for 1.3s
    }
    else
    {
        // do nothing
    }
    
}

// Make this return true when this Command no longer needs to run execute()
bool LeftAuto::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void LeftAuto::End(bool interrupted) {
    m_drive->DriveStop();
    m_shooter->ShooterStop();
    m_intake->IntakeStop();
    m_climber->ClimberStop();
    m_conveyer->ConveyerStop();

    m_firstTime = true;
}

bool LeftAuto::RunsWhenDisabled() const {
    return false;
}