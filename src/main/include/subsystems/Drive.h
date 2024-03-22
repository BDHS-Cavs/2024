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

#include <frc/drive/DifferentialDrive.h>
//#include <frc/motorcontrol/MotorControllerGroup.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
//#include <frc/Encoder.h>
#include <frc/ADXRS450_Gyro.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/WaitCommand.h>
#include <frc/BuiltInAccelerometer.h>

#include <units/angle.h>
#include <units/length.h>

#include <Constants.h>

class Drive: public frc2::SubsystemBase {

private:

    
    
public:
    Drive();
	
    void Periodic() override;
    void SimulationPeriodic() override;
    void Motivate(double leftSpeed, double rightSpeed);
    void AutoMotivateRotateLeft();
    void AutoMotivateRotateRight();
    void AutoMotivateBackward();
    void DriveStop();
    bool CompareAngles(double x, double y, double epsilon);
    bool CalculateAverageEncoderDistance();
    void AutoMotivateForward();

    frc::ADXRS450_Gyro m_drivegyro;
    
    // left
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftFront{DriveConstants::kFrontLeftMotorPort};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_leftRear{DriveConstants::kRearLeftMotorPort};


    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_turningLeftFront{DriveConstants::kFrontLeftTurningMotorPort};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_turningLeftRear{DriveConstants::kRearLeftTurningMotorPort};


    //right
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightFront{DriveConstants::kFrontRightMotorPort};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_rightRear{DriveConstants::kRearRightMotorPort};

    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_turningRightFront{DriveConstants::kFrontRightTurningMotorPort};
    ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_turningRightRear{DriveConstants::kRearRightTurningMotorPort};

    //frc::MotorControllerGroup m_controllerRight{m_leftFront, m_leftRear};
    //frc::MotorControllerGroup m_controllerLeft{m_rightFront, m_rightRear};

    frc::DifferentialDrive m_differentialDrive{m_leftFront, m_rightFront}; //might have to change left and right to make turning work
};