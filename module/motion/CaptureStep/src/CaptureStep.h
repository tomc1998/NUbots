#ifndef MODULE_MOTION_CAPTURESTEP_H
#define MODULE_MOTION_CAPTURESTEP_H

#include <Eigen/Core>
#include <nuclear>

namespace module {
namespace motion {

    class CaptureStep : public NUClear::Reactor {

    private:
        float z0;                    // Desired torso height of CoM
        Eigen::Vector3d com;         // Current position of the CoM in world space
        Eigen::Vector3d com_target;  // Desired position of the CoM in world space
        Eigen::Vector3d zmp;         // Current position of the ZMP in torso space
        Eigen::Vector3d rpy;         // Current roll, pitch, yaw of the CoM from vertical

        enum State { DOUBLE_SUPPORT, SINGLE_SUPPORT, CAPTURING, CAPTURED };
        State step_state;  // State of capture step, either double or single support

    public:
        /// @brief Called by the powerplant to build and setup the CaptureStep reactor.
        explicit CaptureStep(std::unique_ptr<NUClear::Environment> environment);
    };

}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_CAPTURESTEP_H
