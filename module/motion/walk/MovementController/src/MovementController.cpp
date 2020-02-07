#include "MovementController.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "extension/Configuration.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/TorsoTarget.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;
        using message::behaviour::ServoCommand;
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::KinematicsModel;
        using message::motion::TorsoTarget;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateGroundSpace;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::nusight::graph;
        using utility::support::Expression;

        MovementController::MovementController(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Configuration>("MovementController.yaml").then([this](const Configuration& cfg) {
                // Use configuration here from file MovementController.yaml

                // Foot controller config
                foot_controller.config.step_height = cfg["foot"]["step_height"].as<Expression>();
                foot_controller.config.well_width  = cfg["foot"]["well_width"].as<Expression>();
                foot_controller.config.step_steep  = cfg["foot"]["step_steep"].as<Expression>();

                const auto& h = foot_controller.config.step_height;
                const auto& s = foot_controller.config.step_steep;
                const auto& w = foot_controller.config.well_width;

                // Constant for f_x and f_z
                foot_controller.config.c =
                    (std::pow(s, 2 / s) * std::pow(h, 1 / s) * std::pow(s * h + (s * s * h), -1 / s)) / w;

                // Motion controller config
                config.time_horizon = cfg["time_horizon"].as<Expression>();

                config.support_gain    = cfg["gains"]["support_gain"].as<Expression>();
                config.swing_gain      = cfg["gains"]["swing_gain"].as<Expression>();
                config.swing_lean_gain = cfg["gains"]["swing_lean_gain"].as<Expression>();
            });

            on<Trigger<Sensors>, With<KinematicsModel>, With<FootTarget>, With<TorsoTarget>>().then(
                [this](const Sensors& sensors,
                       const KinematicsModel& model,
                       const FootTarget& foot_target,
                       const TorsoTarget& torso_target) {
                    using namespace std::chrono;

                    // Set the support and swing matrices
                    Eigen::Affine3d Hts = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                    Eigen::Affine3d Htw = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

                    // Set the time now so the calculations are consistent across the methods
                    const auto now       = NUClear::clock::now();
                    auto torso_time_left = duration_cast<duration<double>>(torso_target.timestamp - now).count();
                    auto swing_time_left = duration_cast<duration<double>>(foot_target.timestamp - now).count();

                    // Retrieve the target matrices
                    Eigen::Affine3d Ht_tg(torso_target.Ht_tg);
                    Eigen::Affine3d Hw_tg(foot_target.Hw_tg);

                    // Retrieve world matrix
                    Eigen::Affine3d Htworld(sensors.Htw);

                    // If we are within time_horizon of our target, we always make time_horizon our target
                    torso_time_left = torso_time_left < config.time_horizon ? config.time_horizon : torso_time_left;
                    swing_time_left = swing_time_left < config.time_horizon ? config.time_horizon : swing_time_left;

                    double w_gain = foot_target.lift ? config.swing_gain : config.swing_lean_gain;

                    // Get ground space
                    Eigen::Affine3d Htg(calculateGroundSpace(Hts, Htworld.inverse()));

                    // Calculate the next torso and next swing foot positions we are targeting
                    Eigen::Affine3d Ht_ng =
                        torso_controller.next_torso(config.time_horizon, torso_time_left, Htg, Ht_tg);
                    Eigen::Affine3d Hw_ng =
                        foot_controller.next_swing(config.time_horizon, swing_time_left, Htw.inverse() * Htg, Hw_tg);

                    // Perform IK for the support and swing feet based on the target torso position
                    Eigen::Affine3d Ht_nw_n = Ht_ng * Hw_ng.inverse();

                    // Inverse kinematics
                    // By using g here, we are assuming the support foot is flat on the ground,
                    // and if it's not it'll try to make it flat on the ground

                    const Eigen::Affine3d left_foot  = torso_target.is_right_foot_support ? Ht_nw_n : Ht_ng;
                    const Eigen::Affine3d right_foot = torso_target.is_right_foot_support ? Ht_ng : Ht_nw_n;


                    //-----------------------
                    //------------------------

                    Sensors sensors1;
                    sensors1.servo = std::vector<Sensors::Servo>(20);

                    std::vector<std::pair<ServoID, double>> left_leg_joints =
                        utility::motion::kinematics::calculateLegJoints(model, left_foot, LimbID::LEFT_LEG);
                    for (const auto& leg_joint : left_leg_joints) {
                        ServoID servoID;
                        double position;

                        std::tie(servoID, position) = leg_joint;

                        sensors1.servo[servoID].present_position = position;
                    }

                    std::vector<std::pair<ServoID, double>> right_leg_joints =
                        utility::motion::kinematics::calculateLegJoints(model, right_foot, LimbID::RIGHT_LEG);
                    for (const auto& leg_joint : right_leg_joints) {
                        ServoID servoID;
                        double position;

                        std::tie(servoID, position) = leg_joint;

                        sensors1.servo[servoID].present_position = position;
                    }

                    Eigen::Affine3d left_foot_position = utility::motion::kinematics::calculatePosition(
                        model, sensors, ServoID::L_ANKLE_ROLL)[ServoID::L_ANKLE_ROLL];
                    Eigen::Affine3d right_foot_position = utility::motion::kinematics::calculatePosition(
                        model, sensors, ServoID::R_ANKLE_ROLL)[ServoID::R_ANKLE_ROLL];

                    auto ikresult =
                        utility::motion::kinematics::calculateLegJoints(model, left_foot_position, LimbID::LEFT_LEG);

                    double lerror = (left_foot_position.matrix().array() - left_foot.matrix().array()).abs().maxCoeff();
                    double rerror =
                        (right_foot_position.matrix().array() - right_foot.matrix().array()).abs().maxCoeff();

                    if (lerror < 1e-4 || rerror < 1e-4) {
                        log("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR "
                            "ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR "
                            "ERROR ERROR ERROR ERROR ERROR");
                        log("Calculating forward kinematics");
                        log("L Error: ", lerror, ", R Error, ", rerror);
                        log("Forward Kinematics predicts left foot: \n", left_foot_position.matrix());
                        log("Forward Kinematics predicts right foot: \n", right_foot_position.matrix());
                        log("Compared to request: \n", left_foot.matrix(), "\n", right_foot.matrix());
                        for (size_t servo_id = 0; servo_id < ServoID::NUMBER_OF_SERVOS; ++servo_id) {
                            log(ServoID(servo_id), ": ", sensors1.servo[servo_id].present_position);
                        }

                        for (int i = 0; i < ikresult.size(); i++) {

                            log(ikresult.at(i).first, " : ", ikresult.at(i).second);
                        }
                    }

                    //-----------------------
                    //------------------------

                    auto left_joints  = calculateLegJoints(model, left_foot, LimbID::LEFT_LEG);
                    auto right_joints = calculateLegJoints(model, right_foot, LimbID::RIGHT_LEG);

                    auto projected_time =
                        time_point_cast<NUClear::clock::duration>(now + duration<double>(config.time_horizon));

                    // Look through each servo
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                    for (const auto& joint : left_joints) {
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                projected_time,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? w_gain : config.support_gain,
                                                100);
                    }


                    for (const auto& joint : right_joints) {
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                projected_time,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? config.support_gain : w_gain,
                                                100);
                    }

                    // Emit our locations to move to
                    emit(std::move(waypoints));
                });
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
