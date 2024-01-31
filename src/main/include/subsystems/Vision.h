#include "Constants.h"

#include <photon/PhotonCamera.h>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>

class Vision: public frc2::SubsystemBase {

private:
    photon::PhotonCamera camera{"LimelightTest"}; 
public:
    Vision();

    void Periodic() override;
    void SimulationPeriodic() override;
    void Scan();
    void Track();
};