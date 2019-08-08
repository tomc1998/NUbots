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
        using utility::motion::kinematics::calculateLegJoints;
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

                // Torso controller config
                torso_controller.config.max_translation = cfg["torso"]["max_translation"].as<Expression>();
                torso_controller.config.max_rotation    = cfg["torso"]["max_rotation"].as<Expression>();


                // Motion controller config
                config.time_horizon     = cfg["time_horizon"].as<Expression>();
                config.projection_angle = cfg["projection_angle"].as<Expression>();
                config.min_rotation     = cfg["min_rotation"].as<Expression>();

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

                    // Set the time now so the calculations are consistent across the methods
                    const auto now       = NUClear::clock::now();
                    auto torso_time_left = duration_cast<duration<double>>(torso_target.timestamp - now).count();
                    auto swing_time_left = duration_cast<duration<double>>(foot_target.timestamp - now).count();

                    // Retrieve the target matrices
                    Eigen::Affine3d Ht_tg(torso_target.Ht_tg);
                    Eigen::Affine3d Hw_tg(foot_target.Hw_tg);

                    // If we are within time_horizon of our target, we always make time_horizon our target
                    torso_time_left = torso_time_left < config.time_horizon ? config.time_horizon : torso_time_left;
                    swing_time_left = swing_time_left < config.time_horizon ? config.time_horizon : swing_time_left;

                    double w_gain = foot_target.lift ? config.swing_gain : config.swing_lean_gain;

                    // Find the support foot to torso transformation matrix, and the swing foot to torso transformation
                    // matrix
                    Eigen::Affine3d Hts = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                    Eigen::Affine3d Htw = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

                    //---------------------------------------------------------------------------------------------
                    //----------------------- CREATE GROUND SPACE -------------------------------------------------
                    //---------------------------------------------------------------------------------------------

                    // // Retrieve rotations needed for creating the space
                    // // support foot to torso rotation, and world to torso rotation
                    Eigen::Matrix3d Rts     = Hts.rotation();
                    Eigen::Matrix3d Rtworld = (Eigen::Affine3d(sensors.Htw)).rotation();

                    // World to support foot
                    Eigen::Matrix3d Rworlds = Rtworld.transpose() * Rts;

                    // Dot product of z with identity z
                    double alpha = std::acos(Rworlds(2, 2));

                    Eigen::Vector3d axis = Rworlds.col(2).cross(Eigen::Vector3d::UnitZ()).normalized();

                    // Axis angle is ground to support foot
                    Eigen::Matrix3d Rwg = Eigen::AngleAxisd(alpha, axis).toRotationMatrix() * Rworlds;
                    Eigen::Matrix3d Rtg = Rtworld * Rwg;

                    // log("\nRtg:\n", Rtg.matrix());

                    // Ground space assemble!
                    Eigen::Affine3d Htg;
                    Htg.linear()      = Rtg;
                    Htg.translation() = Hts.translation();

                    // ------------------------------------------------------
                    // ------------------------------------------------------

                    // Calculate the next torso and next swing foot positions we are targeting
                    Eigen::Affine3d Ht_ng =
                        torso_controller.next_torso(config.time_horizon, torso_time_left, Htg, Ht_tg);

                    // log("\ntime horizon:",
                    //     config.time_horizon,
                    //     "\nswing time left:",
                    //     swing_time_left,
                    //     "\nHtw.i\n",
                    //     Htw.inverse().matrix(),
                    //     "\nHtg\n",
                    //     Htg.matrix(),
                    //     "\nHwg\n",
                    //     (Htw.inverse() * Htg).matrix(),
                    //     "\nHw_tg\n",
                    //     Hw_tg.matrix());

                    Eigen::Affine3d Hw_ng =
                        foot_controller.next_swing(config.time_horizon, swing_time_left, Htw.inverse() * Htg, Hw_tg);

                    // Perform IK for the support and swing feet based on the target torso position
                    Eigen::Affine3d Ht_nw_n = Ht_ng * Hw_ng.inverse();


                    // log("\nMOVEMENT CONTROLLER LOGS:\n",
                    //     "torso time left:",
                    //     torso_time_left,
                    //     "\nHt_tg:\n",
                    //     convert(Ht_tg.matrix()),
                    //     "\nGround space check:",
                    //     ((Rtg * Eigen::Vector3d::UnitZ()) - (Rtworld * Eigen::Vector3d::UnitZ())).transpose(),
                    //     "\nEND MOVEMENT CONTROLLER LOGS.");

                    Eigen::Affine3d Ht_ns = (Ht_ng * Htg.inverse()) * Hts;


                    // ------------------------------------------------------
                    // ---------TESTING: REMOVING FOOT KINEMATICS------------
                    // ------------------------------------------------------

                    // const Transform3D t_t = convert(Ht_ng.matrix());
                    // // Retrieve joint positions from inverse kinematics
                    // auto joints = torso_target.is_right_foot_support ? calculateLegJoints(model, t_t,
                    // LimbID::RIGHT_LEG)
                    //                                                  : calculateLegJoints(model, t_t,
                    //                                                  LimbID::LEFT_LEG);


                    // ------------------------------------------------------
                    // ------------------------------------------------------

                    // Inverse kinematics
                    // By using g here, we are assuming the support foot is flat on the ground,
                    // and if it's not it'll try to make it flat on the ground

                    const Transform3D t_t         = convert(Ht_ng.matrix());
                    const Transform3D t_w         = convert(Ht_nw_n.matrix());
                    const Transform3D& left_foot  = torso_target.is_right_foot_support ? t_w : t_t;
                    const Transform3D& right_foot = torso_target.is_right_foot_support ? t_t : t_w;


                    auto left_joints  = calculateLegJoints(model, left_foot, LimbID::LEFT_LEG);
                    auto right_joints = calculateLegJoints(model, right_foot, LimbID::RIGHT_LEG);

                    auto foot_time =
                        time_point_cast<NUClear::clock::duration>(now + duration<double>(config.time_horizon));

                    // Look through each servo
                    auto waypoints = std::make_unique<std::vector<ServoCommand>>();

                    for (const auto& joint : left_joints) {
                        // waypoints->emplace_back(
                        //     foot_target.subsumption_id,
                        //     NUClear::clock::time_point(NUClear::clock::duration(0)),
                        //     joint.first,
                        //     joint.second,
                        //     torso_target.is_right_foot_support ? config.swing_gain : config.support_gain,
                        //     100);
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                now,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? w_gain : config.support_gain,
                                                100);
                    }


                    for (const auto& joint : right_joints) {
                        // waypoints->emplace_back(
                        //     foot_target.subsumption_id,
                        //     NUClear::clock::time_point(NUClear::clock::duration(0)),
                        //     joint.first,
                        //     joint.second,
                        //     torso_target.is_right_foot_support ? config.swing_gain : config.support_gain,
                        //     100);
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                now,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? config.support_gain : w_gain,
                                                100);
                    }

                    // Emit our locations to move to
                    emit(waypoints);
                });
        }  // namespace walk
    }      // namespace walk
}  // namespace motion
}  // namespace module
