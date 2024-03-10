#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain(SwerveModule& fl, SwerveModule& fr, SwerveModule& bl, SwerveModule& br)
: m_fl(fl), m_fr(fr), m_bl(bl), m_br(br)
{
};

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

        m_br.Drive(brSpeed, brAngle);
        m_bl.Drive(blSpeed, blAngle);
        m_fr.Drive(frSpeed, frAngle);
        m_fl.Drive(flSpeed, flAngle);
    }


    void Drivetrain::DrivetrainStop() {
    m_fl.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_fr.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_bl.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_br.m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);

    m_fl.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_fr.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_bl.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_br.m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
}