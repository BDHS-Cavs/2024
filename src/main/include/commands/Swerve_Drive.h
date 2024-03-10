#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Robot.h"

class Swerve_Drive: public frc2::CommandHelper<frc2::Command, Swerve_Drive> {

public:

    explicit Swerve_Drive(Drivetrain* m_swerve);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
    bool RunsWhenDisabled() const override;

private:
    Drivetrain* m_swerve;
    Robot* m_robot; //have to do this for this command only because the way nick did it was bringing robot into the command (usually we bring the command into robotcontainer)
};