#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int turningMotorPort, int driveMotorPort, int turningAnalogEncoderPort)
                          : m_turningMotor(turningMotorPort),
                            m_driveMotor(driveMotorPort),
                            m_turningAnalogEncoder(turningAnalogEncoderPort)
{
    m_driveMotor.ConfigPeakOutputForward(1);
    m_driveMotor.ConfigPeakOutputReverse(-1);

    m_turningMotor.Config_kP(0, DriveConstants::angleP, DriveConstants::TIMEOUT);
    m_turningMotor.Config_kI(0, DriveConstants::angleI, DriveConstants::TIMEOUT);
    m_turningMotor.Config_kD(0, DriveConstants::angleD, DriveConstants::TIMEOUT);
    m_turningMotor.Config_kF(0, DriveConstants::angleF, DriveConstants::TIMEOUT);
    //"Try not to use this if possible and if you do wait until PIDF have been set." m_turningMotor.Config_IntegralZone(0, 50, DriveConstants::TIMEOUT);
    //"These two are needed if you're using motion magic for setting the angle motor"
    //"If you do, calculate F, V, A, and then P, D, and then I."
    m_turningMotor.ConfigMotionCruiseVelocity(DriveConstants::angleV, DriveConstants::TIMEOUT);
    m_turningMotor.ConfigMotionAcceleration(DriveConstants::angleA, DriveConstants::TIMEOUT);
    m_turningMotor.SetInverted(true);

    //m_driveMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
    //m_turningMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

    m_driveMotor.ConfigNominalOutputForward(0.9166, 0);//% delay to wait for the error code, cyborg's numbers. TODO
    m_driveMotor.ConfigNominalOutputReverse(-0.9166, 0);
    m_driveMotor.ConfigContinuousCurrentLimit(40, DriveConstants::TIMEOUT);
    m_driveMotor.ConfigPeakCurrentLimit(45, DriveConstants::TIMEOUT);
    m_driveMotor.ConfigPeakCurrentDuration(250, DriveConstants::TIMEOUT);

    m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
};

void SwerveModule::Drive(double speed, double angle)
{
  angle *= DriveConstants::COUNTPERDEG;

    m_driveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
    m_turningMotor.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, angle);
}

int SwerveModule::getAnglePosition() {
    //return m_turningMotor.GetSelectedSensorPosition(0);
    return m_turningAnalogEncoder.GetAbsolutePosition();
}

int SwerveModule::getDrivePosition() {
    //return m_driveMotor.GetSelectedSensorPosition(0);
}