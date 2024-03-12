// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CenterAuto.h"

#include <cmath>

CenterAuto::CenterAuto(Drive* m_drive, Shooter* m_shooter, Conveyer* m_conveyer)
    : m_drive(m_drive), m_shooter(m_shooter), m_conveyer(m_conveyer) {
  AddRequirements(m_drive);
  AddRequirements(m_shooter);
  AddRequirements(m_conveyer);
}

void CenterAuto::Initialize() {

}

void CenterAuto::Execute() {

}

void CenterAuto::End(bool interrupted) {

}

bool CenterAuto::IsFinished() {

}