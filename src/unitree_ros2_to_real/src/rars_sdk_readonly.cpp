#include "rars_bridge.hpp"
#include <cmath>
#include <stdexcept>
namespace sim2real {
SdkRarsReadOnlyTransport::SdkRarsReadOnlyTransport(rars_arm::RarsArm& owner,const SerialOwnerLease& lease)
 :owner_(owner),lease_(lease) {
 if(!lease_.acquired())throw std::invalid_argument("Serial owner must hold the shared device lease");
}
std::optional<RarsFrame> SdkRarsReadOnlyTransport::Read(SafetyTime now) {
 if(!lease_.acquired())return std::nullopt;
 RarsFrame frame;
 if(!owner_.tryReadJointState(frame.joints))return std::nullopt;
 frame.communication=owner_.communicationStatus();frame.received=now;return frame;
}
void SdkRarsReadOnlyTransport::RecordAcceptedTarget(const std::array<float,6>& q,SafetyTime when) {
 for(float v:q)if(!std::isfinite(v))throw std::invalid_argument("Nonfinite accepted arm target");
 accepted_=AcceptedArmTarget{q,when,true};
}
} // namespace sim2real
