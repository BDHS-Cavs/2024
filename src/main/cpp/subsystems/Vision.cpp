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
    
} 

void Vision::Track(){
    
}
