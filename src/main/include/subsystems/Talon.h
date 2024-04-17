#pragma once

#include <numbers>

#include "Constants.h"

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"

class Talon {
 public:
  Talon(int deviceID, double gearRatio, ControlMode controlMode, Encoder encoder, boolean flippedSensor, double protectedZoneStart, double protectedZoneSize);


	ctre::phoenix::motorcontrol::ControlMode controlMode;

	ctre::phoenix::motorcontrol::ControlMode current = ctre::phoenix::motorcontrol::ControlMode::Current;
	ctre::phoenix::motorcontrol::ControlMode follower = ctre::phoenix::motorcontrol::ControlMode::Follower;
	ctre::phoenix::motorcontrol::ControlMode percent = ctre::phoenix::motorcontrol::ControlMode::PercentOutput;
	ctre::phoenix::motorcontrol::ControlMode position = ctre::phoenix::motorcontrol::ControlMode::Position;
	ctre::phoenix::motorcontrol::ControlMode velocity = ctre::phoenix::motorcontrol::ControlMode::Velocity;
	ctre::phoenix::motorcontrol::ControlMode disabled = ctre::phoenix::motorcontrol::ControlMode::Disabled;
	ctre::phoenix::motorcontrol::NeutralMode brake = ctre::phoenix::motorcontrol::NeutralMode::Brake;
	ctre::phoenix::motorcontrol::NeutralMode coast = ctre::phoenix::motorcontrol::NeutralMode::Coast;

	Compass compass;
	Convert convert;
	bool hasEncoder;
	
	bool updated = false;
	double lastSetpoint = 0.0;
	double lastLegalDirection = 1.0;
	Logger logger;

};