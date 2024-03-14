/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "Constants.h"

#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>
#include <photon/PhotonPoseEstimator.h>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>

#include <utility>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

class Vision: public frc2::SubsystemBase {

private:

    //TODO photon::PhotonCamera camera{"limelightcam"};
    //photon::PhotonPipelineResult result;;
    //bool hasTargets;
    //double yaw;
    //double pitch;
    //double area;
    //double skew;
    //int apriltagID;


    //TODO: remove? double yaw = target.GetYaw();
    //TODO: remove? double pitch = target.GetPitch();
    //TODO: remove? double area = target.GetArea();
    //TODO: remove? double skew = target.GetSkew();

//TODO: remove? photon::PhotonPoseEstimator m_poseEstimator{
//TODO: remove?       frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
//TODO: remove?       photon::MULTI_TAG_PNP_ON_RIO, std::move(photon::PhotonCamera{"OV5647"}),
//TODO: remove?       frc::Transform3d{}};
//TODO: remove? 
//TODO: remove?   inline std::optional<photon::EstimatedRobotPose> Update(
//TODO: remove?       frc::Pose2d estimatedPose) {
//TODO: remove?     m_poseEstimator.SetReferencePose(frc::Pose3d(estimatedPose));
//TODO: remove?     return m_poseEstimator.Update();
//TODO: remove?   }

public:
    Vision();

    void Periodic() override;
    void SimulationPeriodic() override;
    //void VisionScan();
    //void VisionTrack();
};