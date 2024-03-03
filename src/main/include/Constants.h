// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
//driving ports (TalonSRX)
constexpr int kFrontLeftDriveMotorPort = 3;  //sm4
constexpr int kRearLeftDriveMotorPort = 5;   //sm3
constexpr int kFrontRightDriveMotorPort = 9; //sm1
constexpr int kRearRightDriveMotorPort = 7;  //sm2

constexpr int kFrontLeftTurningMotorPort = 4;
constexpr int kRearLeftTurningMotorPort = 6;
constexpr int kFrontRightTurningMotorPort = 10;
constexpr int kRearRightTurningMotorPort = 2;

constexpr int kFrontLeftTurningEncoderChannelA = 2;
constexpr int kRearLeftTurningEncoderChannelA = 6;
constexpr int kFrontRightTurningEncoderChannelA = 12;
constexpr int kRearRightTurningEncoderChannelA = 15;

constexpr int kFrontLeftTurningEncoderChannelB = 3;
constexpr int kRearLeftTurningEncoderChannelB = 7;
constexpr int kFrontRightTurningEncoderChannelB = 13;
constexpr int kRearRightTurningEncoderChannelB = 14;

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = true;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = true;

constexpr int kFrontLeftDriveEncoderChannelA = 0;
constexpr int kRearLeftDriveEncoderChannelA = 4;
constexpr int kFrontRightDriveEncoderChannelA = 10;
constexpr int kRearRightDriveEncoderChannelA = 16;

constexpr int kFrontLeftDriveEncoderChannelB = 1;
constexpr int kRearLeftDriveEncoderChannelB = 5;
constexpr int kFrontRightDriveEncoderChannelB = 11;
constexpr int kRearRightDriveEncoderChannelB = 17;

constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = true;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
constexpr auto ks = 1_V; //volts
constexpr auto kv = 0.8 * 1_V * 1_s / 1_m; //volts * seconds / meters
constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m; //volts * seconds^2 / meters

// Example value only - as above, this must be tuned for your drive!
constexpr double kPFrontLeftVel = 0.5;
constexpr double kPRearLeftVel = 0.5;
constexpr double kPFrontRightVel = 0.5;
constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ClimberConstants {
//climber (SparkMax)
constexpr int kClimberMotor1Port = 14; //spark
constexpr int kClimberMotor2Port = 15; //spark
}

namespace IntakeConstants {
//intake (TalonSRX)
constexpr int kIntakeMotorPort = 1;
}

namespace ShooterConstants {
//shooter (SparkMax)
constexpr int kShooterMotor1Port = 12; //spark
constexpr int kShooterMotor2Port = 13; //spark
}

namespace ConveyerConstants {
//conveyer (TalonSRX)
constexpr int kConveyerMotorPort = 8;
}

namespace VisionConstants {
//vision (limelight)
//constexpr char kCameraName[5] = "OV5647";

}

namespace ModuleConstants {
constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = 0.15;
constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (kWheelDiameterMeters * std::numbers::pi) /
    static_cast<double>(kEncoderCPR);

constexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 1;
constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
