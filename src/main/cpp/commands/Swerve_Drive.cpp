#include "commands/Swerve_Drive.h"
Swerve_Drive::Swerve_Drive(Drivetrain* m_swerve)
:m_swerve(m_swerve){

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("Swerve_Drive");
    //AddRequirements(m_swerve);
}

// Called just before this Command runs the first time
void Swerve_Drive::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void Swerve_Drive::Execute() {
       m_swerve->calculateVectors(m_robot->m_container->getX(), m_robot->m_container->getY(), m_robot->m_container->getZ());
}

// Make this return true when this Command no longer needs to run execute()
bool Swerve_Drive::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void Swerve_Drive::End(bool interrupted) {
    //TODO do we need? m_swerve->DrivetrainStop();
}

bool Swerve_Drive::RunsWhenDisabled() const {
    return false;
}