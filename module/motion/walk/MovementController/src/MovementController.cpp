#include "MovementController.h"

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
#include "utility/math/comparison.h"
#include "utility/math/matrix/Transform3D.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"

namespace module {
namespace motion {
    namespace walk {

        using extension::Configuration;

        using message::behaviour::ServoCommand;
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::KinematicsModel;
        using message::motion::TorsoTarget;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::almost_equal;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::motion::kinematics::legPoseValid;
        using utility::nusight::graph;


        /* Naming conventions
         * Vectors: rABc, a vector from point 'B' to point 'A' in space 'c'.
         * Transformation matrices: Hab, from the space 'b' to the space 'a'.
         * Hab.linear = Rab, rotation from space 'b' to space 'a'.
         * Hab.translation = rBAa, vector from point 'A' to point 'B' in space 'a'.
         *
         * Hab * Hbc = Hac ('inner' letters/spaces cancel)
         * Rab * Rbc = Rac
         * Hab * rCBb = rCAa
         */

        double MovementController::f_x(const Eigen::Vector3d& pos) {
            return (-pos.x() / std::abs(pos.x())) * std::exp(-std::abs(std::pow(c * pos.x(), -step_steep)));
        }

        double MovementController::f_z(const Eigen::Vector3d& pos) {
            return std::exp(-std::abs(std::pow(c * pos.x(), -step_steep))) - (pos.z() / step_height);
        }

        /**
         * Calculates the next torso postion we should be at in time_horizon amount of time
         *
         * @param time_left The amount of time remaining until we should hit our target
         * @param Htg homogeneous transfrom from ground space to current torso space
         * @param Ht_tg homogeneous transform from ground space to our target torso space
         * @return Ht_ng homogeneous transfrom from ground space to the horizon torso space (next torso space)
         */
        Eigen::Affine3d MovementController::next_torso(const double time_left,
                                                       const Eigen::Affine3d& Htg,
                                                       const Eigen::Affine3d& Ht_tg) {

            // Ht_ng will be the transformation matrix we will return
            Eigen::Affine3d Ht_ng;

            Eigen::Affine3d Htt_t  = Htg * Ht_tg.inverse();
            Eigen::Vector3d rT_tTt = Htt_t.translation();
            Eigen::Vector3d rNTt   = rT_tTt.normalized() * (rT_tTt.norm() / time_left) * time_horizon;
            Eigen::Vector3d rNGg   = (Htg.inverse() * rNTt);

            if (rT_tTt.norm() < well_width) {
                rNGg = Ht_tg.inverse().translation();
            }

            double factor = time_horizon / time_left;

            // // Slerp the current rotation and target rotation to get a next rotation
            Ht_ng.linear() = Eigen::Quaterniond(Htg.rotation())
                                 .slerp(factor, Eigen::Quaterniond(Ht_tg.rotation()))
                                 .toRotationMatrix();
            Eigen::Affine3d Hgn;
            Hgn.linear()        = Ht_ng.rotation().inverse();
            Hgn.translation()   = rNGg;
            Ht_ng.translation() = Hgn.inverse().translation();


            bool ridiculous_rotation =
                (Eigen::Matrix3d::Identity() - Htg.rotation() * Ht_ong.rotation().inverse()).norm() > rotation_limit;

            if (Htong_flag && !ridiculous_rotation) {
                // Get a rotation change for ratcheting
                double new_ratchet_rotation =
                    (Eigen::Matrix3d::Identity() - Ht_ng.rotation() * Ht_tg.rotation().transpose()).norm();
                double old_ratchet_rotation =
                    (Eigen::Matrix3d::Identity() - Ht_ong.rotation() * Ht_tg.rotation().transpose()).norm();
                if (old_ratchet_rotation < new_ratchet_rotation) {
                    log("ROTATION RATCHET");
                    Ht_ng.linear() = Ht_ong.linear();
                }
                else {
                    log("ROTATION SUCCESS!");
                }
            }
            else {
                log("RIDICULOUS ROTATION (or not flag");
            }

            bool ridiculous = (Htg.inverse().translation() - Ht_ong.inverse().translation()).norm() > velocity_limit;

            if (Htong_flag && !ridiculous) {
                double old_distance = (Ht_ong.inverse().translation() - Ht_tg.inverse().translation()).norm();
                double new_distance = (Ht_ng.inverse().translation() - Ht_tg.inverse().translation()).norm();
                if (old_distance < new_distance) {
                    log("TRANSLATION RATCHET");
                    Ht_ng.translation() = Ht_ong.translation();
                }
                else {
                    log("TRANSLATION SUCCESS!");
                }
            }
            else {
                log("RIDICULOUS TRANSLATION (or not flag) ");
            }

            Ht_ong     = Ht_ng;
            Htong_flag = true;

            return Ht_ng;
        }


        /**
         * Calculates the next swing foot postion we should be at in time_horizon amount of time
         *
         * @param time_left The amount of time remaining until we should hit our target
         * @param Hf_wg homogeneous transform from ground space to current s"wing" (w) foot space
         * @param Hf_wtg homogeneous transform from ground space to target s"wing" (w) foot space
         *
         * @return Hf_wng homogeneous transform from ground space to the horizon swing space (next swing space)
         */
        Eigen::Affine3d MovementController::next_swing(const double time_left,
                                                       const Eigen::Affine3d& Htg,
                                                       const Eigen::Affine3d& Hwg,
                                                       const Eigen::Affine3d& Hw_tg,
                                                       bool lift) {
            // Hw_ng is the transformation matrix we will return
            Eigen::Affine3d Hw_ng;

            // // If the target is directly in front of the foot, just do vector field with no interpolation


            Eigen::Vector3d rWGg   = Hwg.inverse().translation();
            Eigen::Vector3d rW_tGg = Hw_tg.inverse().translation();
            Eigen::Affine3d Htw    = Htg * Hwg.inverse();

            // //*************A METHOD********************************

            // Eigen::Vector3d rtargetswing;
            // rtargetswing = (rWGg - rW_tGg);

            // Eigen::Vector3d groundz;
            // groundz = Htg.linear().col(2).normalized();
            // Eigen::Vector3d groundy;
            // groundy = Htg.linear().col(1).normalized();
            // Eigen::Vector3d groundx;
            // groundx = Htg.linear().col(0).normalized();


            // Eigen::Vector3d projZ;
            // projZ = rtargetswing.dot(groundz) * groundz;

            // Eigen::Vector3d projXY;
            // projXY = rtargetswing - projZ;

            // Eigen::Vector3d TARGET;
            // TARGET = 0.1 * groundx - 0.1 * groundy;

            // Eigen::Vector3d newposition;
            // // newposition = rWGg + 0.02 * groundz;
            // // newposition = rWGg + 0.02 * (TARGET - rWGg);
            // newposition = 0.03 * Hwg.inverse().linear().col(0).normalized() + rWGg;
            // // if (rtargetswing.norm() < 0.5) {
            // //     newposition = rWGg - 0.05 * groundz - 0.05 * projXY;
            // // }
            // // else {
            // //     newposition = rWGg + 0.05 * groundz - 0.05 * projXY;
            // // }

            // // Eigen::Vector3d field;
            // // field.x() = -projXY.norm();
            // // field.y() = 0;
            // // field.z() = projZ.norm();

            // // Eigen::Vector3d field_output(f_x(field), 0, f_z(field));

            // // field = ((projXY.norm() + projZ.norm()) / time_left) * (time_horizon) *field_output.normalized();
            // // log(field.transpose());
            // // Eigen::Vector3d new_position;
            // // new_position = field.x() * projXY.normalized() + field.z() * groundz;

            // Eigen::Vector3d rNGg = newposition;


            // //*******************DIFFERENT METHOD********************

            // Create matrix Hpg (ground to plane space)
            Eigen::Affine3d Hpg = Hw_tg;
            // Set the rotation of this matrix to identity. This means, the space is rotated to align with ground
            Hpg.linear() = Eigen::Matrix3d::Identity();

            // Create matrix from plane space to swing foot
            Eigen::Affine3d Hwp = Hwg * Hpg.inverse();
            // get the translation of target to swing foot
            Eigen::Vector3d target_to_swing = Hwp.inverse().translation();

            // set the vector to 0 so that we can align the vector field with ground space rather than on an angle
            target_to_swing.z() = 0;

            // get the angle between the x axis and the vector to the swing foot
            double alpha = std::acos(target_to_swing.normalized().dot(Eigen::Vector3d::UnitX()));

            // Rotate the matrix about the z-axis by alpha so target space is now facing the swing foot
            // At this point, we can no longer use Hpg for plane space
            Hwp.rotate(Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()));

            // target to swing foot vector Hpw
            Eigen::Vector3d rWPp = Hwp.inverse().translation();

            // apply vector field to rNTarget
            Eigen::Vector3d vectorfield(f_x(rWPp), 0, f_z(rWPp));
            Eigen::Vector3d rNPp = rWPp + (vectorfield.normalized() * rWPp.norm() * (time_horizon / time_left));

            // Get rNWw then change it to ground space to get rNGg
            Eigen::Vector3d rNGg = Hwg.inverse() * (Hwp * rNPp);


            // //*******************************************************
            // //**************ANOTHER METHOD***************************
            // //*******************************************************
            // Eigen::Affine3d Htn;
            // double factor;
            // factor = time_horizon / time_left;

            // Eigen::Vector3d newposition;
            // newposition = rWGg + factor*(rW_tGg - rWGg);

            // Eigen::Vector3d groundz;
            // groundz = Htg.linear().col(2).normalized();
            // Eigen::Vector3d projZ;
            // projZ = (rWGg - rW_tGg).dot(groundz) * groundz;
            // Eigen::Vector3d projXY;
            // projXY = (rWGg - rW_tGg) - projZ;

            // Eigen::Vector3d control;
            // control = rW_tGg + step_height * groundz + 0.75 * projXY;

            // //bezier
            // //newposition = std::pow((1-factor),2)*rWGg + 2*(1-factor)*factor*control + std::pow(factor,2)*rW_tGg;


            // Htn.translation() = Htg.rotation() * (newposition - Htg.inverse().translation());
            // Htn.linear()      = Htg.rotation();
            // Hw_ng             = Htn.inverse() * Htg;

            // //*******************************************************


            double current_distance_target = rWPp.norm();
            // log("distance", current_distance_target);

            // If we are very close to the target, just go to the target directly
            if (current_distance_target < well_width || !lift) {
                // log("lift:", lift, "distance to target:", current_distance_target);
                rNGg = rW_tGg;
            }

            // Set the translation of the matrix to the correct vector
            Eigen::Affine3d Hgn;
            Hgn.translation()   = rNGg;
            Hgn.linear()        = Eigen::Matrix3d::Identity();
            Hw_ng.translation() = Hgn.inverse().translation();

            // Hw_ng.translation() = Hw_tg.translation();
            // log("target: ", rW_tGg.transpose());
            Hw_ng.linear() = Eigen::Matrix3d::Identity();

            double factor = time_horizon / time_left;
            // Hw_ng.linear() =
            //     Eigen::Quaterniond(Hwg.linear()).slerp(factor,
            //     Eigen::Quaterniond(Hw_tg.linear())).toRotationMatrix();

            return Hw_ng;
        }


        MovementController::MovementController(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)) {

            on<Configuration>("MovementController.yaml").then([this](const Configuration& config) {
                // Use configuration here from file MovementController.yaml
                step_height = config["step_height"].as<double>();
                well_width  = config["well_width"].as<double>();
                step_steep  = config["step_steep"].as<double>();

                time_horizon   = config["time_horizon"].as<double>();
                velocity_limit = config["velocity_limit"].as<double>();
                rotation_limit = config["rotation_limit"].as<double>();

                if (rotation_limit > 2 * std::sqrt(2)) {
                    rotation_limit = 2 * std::sqrt(2);
                }

                // s_gain = config["s_gain"].as<double>();
                // gain   = config["gain"].as<double>();

                Htong_flag = false;

                // Constant for f_x and f_z
                c = (std::pow(step_steep, 2 / step_steep) * std::pow(step_height, 1 / step_steep)
                     * std::pow(step_steep * step_height + (step_steep * step_steep * step_height), -1 / step_steep))
                    / well_width;
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

                    double swing_gain   = foot_target.gain;
                    double support_gain = torso_target.gain;

                    // If we are within time_horizon of our target, we always make time_horizon our target
                    torso_time_left = torso_time_left < time_horizon ? time_horizon : torso_time_left;
                    swing_time_left = swing_time_left < time_horizon ? time_horizon : swing_time_left;

                    // Find the support foot to torso transformation matrix, and the swing foot to torso transformation
                    // matrix
                    Eigen::Affine3d Hts = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                    Eigen::Affine3d Htw = torso_target.is_right_foot_support
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

                    log(torso_time_left);
                    //---------------------------------------------------------------------------------------------
                    //----------------------- CREATE GROUND SPACE -------------------------------------------------
                    //---------------------------------------------------------------------------------------------

                    // Retrieve rotations needed for creating the space
                    // support foot to torso rotation, and world to torso rotation
                    Eigen::Matrix3d Rts     = Hts.rotation();
                    Eigen::Matrix3d Rtworld = (Eigen::Affine3d(sensors.Htw)).rotation();

                    // World to support foot
                    Eigen::Matrix3d Rsworld = Rts.transpose() * Rtworld;

                    // Dot product of z with identity z
                    double alpha = std::acos(Rsworld(2, 2));

                    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ().cross(Rsworld.col(2)).normalized();

                    // Axis angle is ground to support foot
                    Eigen::Matrix3d Rsg = Eigen::AngleAxisd(alpha, axis).toRotationMatrix();
                    Eigen::Matrix3d Rtg = Rts * Rsg;

                    // Ground space assemble!
                    Eigen::Affine3d Htg;

                    Htg.linear()      = Rtg;
                    Htg.translation() = Hts.translation();


                    //---------------------------------------------------------------------------------------------
                    //---------------------------------------------------------------------------------------------

                    // Calculate the next torso and next swing foot positions we are targeting
                    Eigen::Affine3d Ht_ng = next_torso(torso_time_left, Htg, Ht_tg);

                    Eigen::Affine3d Hw_ng =
                        next_swing(swing_time_left, Ht_ng, (Htw.inverse() * Htg), Hw_tg, foot_target.lift);

                    // Perform IK for the support and swing feet based on the target torso position
                    Eigen::Affine3d Ht_nw_n = Ht_ng * Hw_ng.inverse();

                    // ------------------------------------------------------
                    // ---------TESTING: REMOVING FOOT KINEMATICS------------
                    // ------------------------------------------------------

                    // const Transform3D t_t = convert<double, 4, 4>(Ht_ng.matrix());
                    // Retrieve joint positions from inverse kinematics
                    // auto joints = torso_target.is_right_foot_support ? calculateLegJoints(model, t_t,
                    // LimbID::RIGHT_LEG)
                    //                                                  : calculateLegJoints(model, t_t,
                    //                                                  LimbID::LEFT_LEG);


                    // ------------------------------------------------------
                    // ------------------------------------------------------

                    // Inverse kinematics
                    // By using g here, we are assuming the support foot is flat on the ground,
                    // and if it's not it'll try to make it flat on the ground
                    const Transform3D t_t         = convert<double, 4, 4>(Ht_ng.matrix());
                    const Transform3D t_w         = convert<double, 4, 4>(Ht_nw_n.matrix());
                    const Transform3D& left_foot  = torso_target.is_right_foot_support ? t_w : t_t;
                    const Transform3D& right_foot = torso_target.is_right_foot_support ? t_t : t_w;

                    // Retrieve joint positions from inverse kinematics


                    auto left_joints  = calculateLegJoints(model, left_foot, LimbID::LEFT_LEG);
                    auto right_joints = calculateLegJoints(model, right_foot, LimbID::RIGHT_LEG);
                    auto waypoints    = std::make_unique<std::vector<ServoCommand>>();

                    // Create the target time based on the time horizon
                    const NUClear::clock::time_point target_time =
                        time_point_cast<NUClear::clock::duration>(now + duration<double>(time_horizon));


                    // HACK: By putting these waypoints first, we trick the Controller into dumping future commands
                    // (previous horizon)
                    for (const auto& joint : right_joints) {
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                now,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? support_gain : swing_gain,
                                                100);
                    }

                    for (const auto& joint : left_joints) {
                        waypoints->emplace_back(foot_target.subsumption_id,
                                                now,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? swing_gain : support_gain,
                                                100);
                    }

                    for (const auto& joint : right_joints) {

                        waypoints->emplace_back(foot_target.subsumption_id,
                                                target_time,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? support_gain : swing_gain,
                                                100);
                    }

                    for (const auto& joint : left_joints) {

                        waypoints->emplace_back(foot_target.subsumption_id,
                                                target_time,
                                                joint.first,
                                                joint.second,
                                                torso_target.is_right_foot_support ? swing_gain : support_gain,
                                                100);
                    }

                    // Emit our locations to move to
                    emit(waypoints);
                });
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
