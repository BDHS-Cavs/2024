// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Intake: public frc2::SubsystemBase {

private:
    ctre::phoenix::motorcontrol::can::TalonSRX m_intakeMotor{IntakeConstants::kIntakeMotorPort}; //tuned value
public:
    Intake();

    void Periodic() override;
    void SimulationPeriodic() override;
    void IntakeRun();
    void IntakeExpel();
    void IntakeStop();
};