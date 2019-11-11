#include "CaptureStep.h"

#include "extension/Configuration.h"
#include "utility/input/ServoID.h"
#include "utility/motion/InverseKinematics.h"

#define G 9.80665

namespace module {
namespace motion {

    using extension::Configuration;

    CaptureStep::CaptureStep(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , z0(0.49)
        , com(Eigen::Vector3d::Zero())
        , zmp(Eigen::Vector3d::Zero())
        , step_state(State::DOUBLE_SUPPORT) {

        on<Configuration>("CaptureStep.yaml").then([this](const Configuration& config) {
            // Use configuration here from file CaptureStep.yaml
            z0 = config["z0"].as<float>();
        });

        on<Trigger<Sensors>>.then([this](const Sensors& sensors) {
            // Get translation of CoM
            com = sensors.Htw.block<3, 1>(3, 0);
            // Calculate ZMP based on 3D-LIPM
            zmp = com - (z0 / G) * sensors.accelerometer;
            // Get roll, pitch, yaw based on the gyroscope
            rpy = sensors.angular_position;

            // Apply hip and ankle feedback


            // Check FK, if foot close to ZMP and in capturing phase, transition to double support
            if (step_state == State::CAPTURING) {
                Eigen::Vector3d rTFt = sensors.forward_kinematics[ServoID::L_ANKLE_ROLL].block<3, 1>(3, 0);
                if (rTFt.norm() <= dzmp) {
                    step_state = State::CAPTURED;
                    com_target = com;
                }
            }
        )};

        // on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>, With<Sensors>, Single,
        // Priority::HIGH>.then([this](const Sensors& sensors){

        // });
    }
}  // namespace motion
}  // namespace module
