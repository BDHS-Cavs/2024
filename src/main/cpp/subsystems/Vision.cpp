#include "subsystems/Vision.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/raw_ostream.h> // for wpi outs()

Vision::Vision(){
    SetName("Vision");
    SetSubsystem("Vision");
}

void Vision::Periodic() {
    // Put code here to be run every loop
}

void Vision::SimulationPeriodic() {
}

// Put methods for controlling this subsystem here and call from commands

void Vision::Scan(){ 
        photon::PhotonPipelineResult result = camera.GetLatestResult();
        photon::PhotonTrackedTarget target = result.GetBestTarget();
} 

void Vision::Track(){
        if (result.HasTargets()) {
            frc::SmartDashboard::PutNumber("PhotonVision Target ID", target.GetFiducialId());
        }
}
