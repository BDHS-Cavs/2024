// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

//void Drivetrain::Drive(double speed, double angle) {
//
//    angle *= DriveConstants::COUNTPERDEG;
//
//        m_driveMotor.set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
//        m_turningMotor.set(ControlMode.MotionMagic, angle);





    //TODOSwerveModule getFL() {
    //TODO    return bl;
    //TODO}
//TODO
    //TODOSwerveModule getFR() {
    //TODO    return fr;
    //TODO}
//TODO
    //TODOSwerveModule getBL() {
    //TODO    return bl;
    //TODO}
//TODO
    //TODOSwerveModule getBR() {
    //TODO    return br;
    //TODO}



  //TODO needed? auto states =
  //TODO needed?     m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
  //TODO needed?         fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
  //TODO needed?                             xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
  //TODO needed?                       : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
  //TODO needed?         period));

  //TODO needed? m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  //TODO needed? auto [fl, fr, bl, br] = states;

  //TODO needed? m_frontLeft.SetDesiredState(fl);
  //TODO needed? m_frontRight.SetDesiredState(fr);
  //TODO needed? m_backLeft.SetDesiredState(bl);
  //TODO needed? m_backRight.SetDesiredState(br);

 //m_frontLeft.m_driveEncoder.SetMinRate(3);
 //m_frontRight.m_driveEncoder.SetMinRate(3);
 //m_backLeft.m_driveEncoder.SetMinRate(3);
 //m_backRight.m_driveEncoder.SetMinRate(3);
 //
 //m_frontLeft.m_turningEncoder.SetMinRate(3);
 //m_frontRight.m_turningEncoder.SetMinRate(3);
 //m_backLeft.m_turningEncoder.SetMinRate(3);
 //m_backRight.m_turningEncoder.SetMinRate(3);




//}


    double Drivetrain::getGyro() {
        return m_gyro.GetAngle();
    }


void Drivetrain::calculateVectors(double x, double y, double z) {
        double L = DriveConstants::L;
        double W = DriveConstants::W;
        double r = sqrt((L * L) + (W * W));
        y *= -1;

        if (fieldCentric) {
            double gyro = getGyro() * std::numbers::pi / 180;
            double temp = y * cos(gyro) + x * sin(gyro);
            x = -y * sin(gyro) + x * cos(gyro);
            y = temp;
        }

        double a = x - z * (L / r) + 0;
        double b = x + z * (L / r);
        double c = y - z * (W / r) + 0;
        double d = y + z * (W / r);

        double brSpeed = sqrt((a * a) + (c * c));
        double blSpeed = sqrt((a * a) + (d * d));
        double frSpeed = sqrt((b * b) + (c * c));
        double flSpeed = sqrt((b * b) + (d * d));

        double max = brSpeed;
        if (brSpeed > max) {
            max = brSpeed;
        }
        if (blSpeed > max) {
            max = blSpeed;
        }
        if (frSpeed > max) {
            max = frSpeed;
        }
        if (flSpeed > max) {
            max = flSpeed;
        }

        if (max > 1) {
            brSpeed /= max;
            blSpeed /= max;
            frSpeed /= max;
            flSpeed /= max;
        }

        double brAngle = (atan2(a, c) * 180 / std::numbers::pi);
        double blAngle = (atan2(a, d) * 180 / std::numbers::pi);
        double frAngle = (atan2(b, c) * 180 / std::numbers::pi);
        double flAngle = (atan2(b, d) * 180 / std::numbers::pi);

        br.Drive(brSpeed, brAngle);
        bl.Drive(blSpeed, blAngle);
        fr.Drive(frSpeed, frAngle);
        fl.Drive(flSpeed, flAngle);
    }





//TODO needed? void Drivetrain::UpdateOdometry() {
//TODO needed?   m_odometry.Update(m_gyro.GetRotation2d(),
//TODO needed?                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
//TODO needed?                      m_backLeft.GetPosition(), m_backRight.GetPosition()});
//TODO needed? }

void Drivetrain::DrivetrainStop() {
    fl.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    fr.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    bl.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    br.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

    fl.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    fr.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    bl.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    br.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}