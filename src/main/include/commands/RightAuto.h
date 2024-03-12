// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drive.h"
#include "subsystems/Shooter.h"
#include "subsystems/Conveyer.h"

class RightAuto : public frc2::CommandHelper<frc2::Command, RightAuto> {
 public:
  /**
   * Creates a new DriveDistance.
   *
   * @param drive The number of inches the robot will drive
   * @param shooter The speed at which the robot will drive
   * @param conveyer The drive subsystem on which this command will run
   */
  RightAuto(Drive* m_drive, Shooter* m_shooter, Conveyer* m_conveyer);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drive* m_drive;
  Shooter* m_shooter;
  Conveyer* m_conveyer;
};