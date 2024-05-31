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
#include "commands/CenterAuto.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h> // for wpi outs()

CenterAuto::CenterAuto(Drive* m_drive, Climber* m_climber, Shooter* m_shooter, Intake* m_intake, Vision* m_vision, Conveyer* m_conveyer)
:m_drive(m_drive),
m_climber(m_climber),
m_shooter(m_shooter),
m_intake(m_intake),
m_vision(m_vision),
m_conveyer(m_conveyer)
{

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("CenterAuto");
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
void CenterAuto::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void CenterAuto::Execute() {

    units::second_t period1 = 1_s; //starts at 1 (2s)
    units::second_t period2 = 3_s; //starts at 3 (2s)
    units::second_t period3 = 5_s; //starts at 5 (0.3s)
    units::second_t period4 = 5.3_s; //starts at 5.3 (1.5s)
    units::second_t period5 = 6.8_s; //starts at 6.8 (1.5s)
    units::second_t period6 = 8.3_s; //starts at 8.3 (1.1s)
    units::second_t period7 = 9.4_s; //starts at 9.4 (1.1s)
    units::second_t period8 = 10.5_s; //starts at 10.5 (2s)
    units::second_t period9 = 12.5_s; //starts at 12.5 (1s)
    units::second_t period10 = 13.5_s; //starts at 13.5 (1s)
    units::second_t period11 = 15.0_s; //starts at 15.0 (1.5s)
    //units::second_t period12 = X_s; //starts at X (Xs)
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
    else if(m_timer.Get() >= period3 && m_timer.Get() < period4) //starts at 5 ends at 5.3 (0.3s)
    {
        m_shooter->ShooterStop(); //stop running the shooter
        m_conveyer->ConveyerStop(); //stop running the conveyer
    }
    else if(m_timer.Get() >= period4 && m_timer.Get() < period5) //starts at 5.3 ends at 6.8 (1.5s)
    {
        m_drive->AutoMotivateBackward(); //start driving backwards
        m_intake->IntakeRun(); //start running the intake
    }
    else if(m_timer.Get() >= period5 && m_timer.Get() < period6) //starts at 6.8 ends at 8.3 (1.5s)
    {
        m_drive->AutoMotivateForward(); //start driving forward
    }
    else if(m_timer.Get() >= period6 && m_timer.Get() < period7) //starts at 8.3 ends at 9.4 (1.1s)
    {
            m_drive->DriveStop(); //stop driving
            m_shooter->ShooterShoot(); //start running the shooter
            m_intake->IntakeStop(); //stop running the intake
    }
    else if(m_timer.Get() >= period7 && m_timer.Get() < period8) //starts at 9.4 ends at 10.5 (1.1s)
    {
            m_shooter->ShooterShoot(); //TODO - its already running...
    }
    else if(m_timer.Get() >= period8 && m_timer.Get() < period9) //starts at 10.5 ends at 12.5 (2s)
    {
            m_conveyer->ConveyerForward(); //start running the conveyer forward
    }
    else if(m_timer.Get() >= period9 && m_timer.Get() < period10) //starts at 12.5 ends at 13.5 (1s)
    {
            m_conveyer->ConveyerStop(); //stop running the conveyer
            m_shooter->ShooterStop(); //stop running the shooter
    }
    else if(m_timer.Get() >= period10 && m_timer.Get() < period11) //starts at 13.5 ends at 15.0 (1.5s)
    {
            m_drive->AutoMotivateBackward(); //start driving backwards -- will not stop until end of auto
    }
/*TODO delete    else if(m_timer.Get() >= period11 && m_timer.Get() < period12) //starts at X ends at X (Xs)
    {
            m_drive->DriveStop();
    }*/
    else
    {
        // do nothing
    }
    
}

// Make this return true when this Command no longer needs to run execute()
bool CenterAuto::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void CenterAuto::End(bool interrupted) {
    m_drive->DriveStop();
    m_shooter->ShooterStop();
    m_intake->IntakeStop();
    m_climber->ClimberStop();
    m_conveyer->ConveyerStop();

    m_firstTime = true;
}

bool CenterAuto::RunsWhenDisabled() const {
    return false;
}