// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel//,
                           //const int driveEncoderChannelA,
                           //const int driveEncoderChannelB,
                           //const int turningEncoderChannelA,
                           //const int turningEncoderChannelB
                           )
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel)//,
      //m_driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
      //m_turningEncoder(turningEncoderChannelA, turningEncoderChannelB) 
      {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  //TODO "counts per degrees"? m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
  //TODO "counts per degrees"?                                    kEncoderResolution);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  //TODO "counts per degrees"? m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi /
  //TODO "counts per degrees"?                                      kEncoderResolution);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  //m_turningPIDController.EnableContinuousInput(
  //    -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}//TODO TALON VERSION: CONFIGUREPEAKOUTPUT FORWARD/REVERSE

frc::SwerveModuleState SwerveModule::GetState() const {
  //TODO TALON get rate?????    return {units::meters_per_second_t{m_driveEncoder.GetRate()},
  //TODO TALON GET SELECTED SENSOR POSITION?  CONFIGURE TO RESET EVERY ROTATION???? (is that what getdistance is???)            units::radian_t{m_turningEncoder.GetDistance()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  //TODO TALON GET SELECTED SENSOR POSITION?  CONFIGURE TO RESET EVERY ROTATION???? (is that what getdistance is???)    return {units::meter_t{m_driveEncoder.GetDistance()},
  //TODO TALON GET SELECTED SENSOR POSITION?  CONFIGURE TO RESET EVERY ROTATION???? (is that what getdistance is???)            units::radian_t{m_turningEncoder.GetDistance()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      //TODO TALON GET SELECTED SENSOR POSITION?  CONFIGURE TO RESET EVERY ROTATION???? (is that what getdistance is???)            units::radian_t{m_turningEncoder.GetDistance()}
      };

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  //const auto driveOutput = m_drivePIDController.Calculate(
  //    m_driveEncoder.GetRate(), state.speed.value());
      //TODO replace with WheelModule.java line 136???
      //TODO or replace with swerve.java calculatevectors line 78

  //const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  //const auto turnOutput = m_turningPIDController.Calculate(
  //    units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians());

  //const auto turnFeedforward = m_turnFeedforward.Calculate(
  //    m_turningPIDController.GetSetpoint().velocity);
      //TODO dont even need this

  // Set the motor outputs.
  //m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  //m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
  //TODO drive with percentoutput instead
}
