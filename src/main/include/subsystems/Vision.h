#include "Constants.h"

#include <photon/PhotonCamera.h>
#include <photon/PhotonUtils.h>

#include <units/angle.h>
#include <units/length.h>

#include <frc2/command/SubsystemBase.h>

class Vision: public frc2::SubsystemBase {

private:
    photon::PhotonCamera camera{"OV5647"}; 
    photon::PhotonPipelineResult result;
    photon::PhotonTrackedTarget target = result.GetBestTarget();

    double yaw = target.GetYaw();
    double pitch = target.GetPitch();
    double area = target.GetArea();
    double skew = target.GetSkew();

public:
    Vision();

    void Periodic() override;
    void SimulationPeriodic() override;
    void Scan();
    void Track();
};