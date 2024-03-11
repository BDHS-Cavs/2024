#pragma once

#include "SwerveModule.h"
#include "frc/ADXRS450_Gyro.h"

class Drivetrain {
 public:
  Drivetrain(SwerveModule& fl, SwerveModule& fr, SwerveModule& bl, SwerveModule& br);

  double getGyro();
  void calculateVectors(double x, double y, double z);
  void DrivetrainStop();





  bool fieldCentric = true;
  frc::ADXRS450_Gyro m_gyro;

  SwerveModule& m_fl;
  SwerveModule& m_fr;
  SwerveModule& m_bl;
  SwerveModule& m_br;

 private:


};
