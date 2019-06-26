#include "StaticWalk.h"

#include <Eigen/Geometry>
#include <vector>
#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/TorsoTarget.h"
#include "message/motion/WalkCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/comparison.h"
#include "utility/math/matrix/Transform3D.h"

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
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::TorsoTarget;
        using message::motion::WalkCommand;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::matrix::Transform3D;

        using message::behaviour::ServoCommand;
        using message::input::Sensors;
        using message::motion::FootTarget;
        using message::motion::KinematicsModel;
        using utility::math::almost_equal;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::motion::kinematics::legPoseValid;
        using utility::nusight::graph;

        Eigen::Affine3d StaticWalk::getGroundSpace(Eigen::Affine3d& Hts, Eigen::Affine3d& Htworld) {
            Eigen::Matrix3d Rts     = Hts.rotation();
            Eigen::Matrix3d Rtworld = Htworld.rotation();

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
            return Htg;
        }


        StaticWalk::StaticWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

            on<Configuration>("StaticWalk.yaml").then([this](const Configuration& config) {
                start_phase = NUClear::clock::now();
                state       = INITIAL;

                // Use configuration here from file StaticWalk.yaml
                torso_height   = config["torso_height"].as<double>();
                stance_width   = config["stance_width"].as<double>();
                phase_time     = std::chrono::milliseconds(config["phase_time"].as<int>());
                double x_speed = config["x_speed"].as<double>();
                double y_speed = config["y_speed"].as<double>();
                double angle   = config["angle"].as<double>();

                y_offset = config["y_offset"].as<double>();
                x_offset = config["x_offset"].as<double>();

                start_right_lean = config["start_right_lean"].as<bool>();

                rotation_limit = 15 * M_PI / 180;

                // Multiply by 2 so that the lean states are accounted for
                time = std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count() * 2;

                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(x_speed, y_speed, angle)));
            });

            on<Trigger<Sensors>, With<WalkCommand>>().then([this](const Sensors& sensors,
                                                                  const WalkCommand& walkcommand) {
                // INITIAL state occurs only as the first state in the walk to set the matrix Hff_s
                if (state == INITIAL) {
                    // Set the matrix Hff_s and change the state to a lean


                    state = start_right_lean ? RIGHT_LEAN : LEFT_LEAN;
                    // log(state);
                    // state = LEFT_LEAN;

                    Eigen::Affine3d Hts = state == LEFT_LEAN
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);
                    Eigen::Affine3d Htworld(sensors.Htw);
                    Eigen::Affine3d Htg = getGroundSpace(Hts, Htworld);

                    Hwg = state == LEFT_LEAN
                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse() * Htg
                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse() * Htg;
                }

                // When the time is over for this phase, begin the next phase
                if (NUClear::clock::now() > start_phase + phase_time) {
                    // Reset the start phase time for the new phase
                    start_phase = NUClear::clock::now();
                    // Change the state of the walk based on what the previous state was
                    switch (state) {
                        case LEFT_LEAN: state = RIGHT_STEP; break;
                        case RIGHT_STEP: {
                            // Store where support is relative to swing
                            // log("rightstep");

                            Eigen::Affine3d Hts(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);
                            Eigen::Affine3d Htworld(sensors.Htw);
                            Eigen::Affine3d Htg = getGroundSpace(Hts, Htworld);


                            // Hff_s = (sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse()
                            //         * (sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                            // or
                            // supportfoot (left) to swingfoot
                            // Hff_s = (sensors.forward_kinematics[ServoID::R_ANKLE_ROLL])
                            //*(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse();
                            // swingfoot space to ground space
                            Hwg = Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse() * Htg;

                            state = RIGHT_LEAN;
                        } break;
                        case RIGHT_LEAN: state = LEFT_STEP; break;
                        case LEFT_STEP: {
                            // Store where support is relative to swing
                            // Hff_s = (sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse()
                            //        * (sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);


                            // swingfoot (left) to supportfoot (right)
                            // log("leftstep");
                            // Support foot *right) to swingfoot (left)
                            // This works better than other way.
                            // Hff_s = (sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse()
                            //         * (sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

                            Eigen::Affine3d Hts(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                            Eigen::Affine3d Htworld(sensors.Htw);
                            Eigen::Affine3d Htg = getGroundSpace(Hts, Htworld);


                            Hwg = Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse() * Htg;

                            state = LEFT_LEAN;
                        } break;
                        default: break;
                    }
                }

                // Put our COM over the correct foot or move foot to target, based on which state we are in
                switch (state) {
                    case LEFT_LEAN: {
                        // Create matrix for TorsoTarget
                        Eigen::Affine3d Hts(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]);
                        Eigen::Affine3d Htworld(sensors.Htw);

                        Eigen::Affine3d Htg(getGroundSpace(Hts, Htworld));

                        Eigen::Vector3d rCTt(sensors.centre_of_mass.x(), sensors.centre_of_mass.y(), 0);
                        Eigen::Vector3d rCGg(Htg.inverse() * rCTt);

                        Eigen::Vector3d rT_tGg(Eigen::Vector3d(x_offset, y_offset, torso_height));

                        rT_tGg = rT_tGg - (rCGg - Htg.inverse().translation());

                        Eigen::Affine3d Hgt_t;
                        Hgt_t.linear()      = Eigen::Matrix3d::Identity();
                        Hgt_t.translation() = rT_tGg;
                        // Hgt_t.translation() = Eigen::Vector3d(0, 0, 0.45);

                        Eigen::Affine3d Ht_tg = Hgt_t.inverse();

                        // Move the torso over the left foot
                        emit(std::make_unique<TorsoTarget>(
                            start_phase + phase_time, false, Ht_tg.matrix(), subsumptionId));


                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, true, Hwg.matrix(), false, subsumptionId));
                    } break;
                    case RIGHT_LEAN: {
                        // Create matrix for TorsoTarget
                        Eigen::Affine3d Hts(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);
                        Eigen::Affine3d Htworld(sensors.Htw);

                        Eigen::Affine3d Htg(getGroundSpace(Hts, Htworld));

                        Eigen::Vector3d rCTt(sensors.centre_of_mass.x(), sensors.centre_of_mass.y(), 0);
                        Eigen::Vector3d rCGg(Htg.inverse() * rCTt);

                        Eigen::Vector3d rT_tGg(Eigen::Vector3d(x_offset, -y_offset, torso_height));

                        rT_tGg = rT_tGg - (rCGg - Htg.inverse().translation());

                        // rT_tGg.y() = rT_tGg.y() > y_limit ? y_limit : rT_tGg.y();

                        Eigen::Affine3d Hgt_t;
                        Hgt_t.linear()      = Eigen::Matrix3d::Identity();
                        Hgt_t.translation() = rT_tGg;
                        // Hgt_t.translation() = Eigen::Vector3d(0, 0, 0.45);

                        Eigen::Affine3d Ht_tg = Hgt_t.inverse();

                        // Move the torso over the left foot
                        emit(std::make_unique<TorsoTarget>(
                            start_phase + phase_time, true, Ht_tg.matrix(), subsumptionId));

                        // Maintain left foot position while the torso moves over the right foot
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, false, Hwg.matrix(), false, subsumptionId));
                    } break;


                    case RIGHT_STEP: {
                        // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
                        // radians/seconds
                        // log("rightstep2");
                        double rotation = walkcommand.command.z();
                        Eigen::Vector3d translation =
                            Eigen::Vector3d(walkcommand.command.x(), walkcommand.command.y(), 0);
                        Eigen::Affine3d Haf;
                        Eigen::Vector3d rASs;

                        // TODO: write stop function and proper check
                        // If there is no rotation to be done, just set the translation to x and y, and set the
                        // rotation to identity
                        // rotation check is to stop the robot from turning outside acceptable parameters
                        if (rotation == 0 || rotation > rotation_limit) {
                            Eigen::Vector3d target = translation * time;
                            target.y() -= stance_width;
                            Haf.linear() = Eigen::Matrix3d::Identity();
                            Eigen::Affine3d Hfa;
                            Hfa.linear()      = Eigen::Matrix3d::Identity();
                            Hfa.translation() = target;
                            Haf.translation() = Hfa.inverse().translation();
                        }


                        // If there is rotation, adjust the translation and rotation for this
                        else {
                            //  Multiply by phase time so that we are moving in x metres/second and y metres/second
                            double radius          = translation.norm() / std::abs(rotation);
                            Eigen::Vector3d origin = Eigen::Vector3d(-translation.y(), translation.x(), 0);
                            origin /= rotation;
                            Eigen::Vector3d end_point =
                                rotation > 0 ? (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                                   + (origin * (1 - std::cos(std::abs(rotation) * time)))
                                             : (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                                   + (origin * (1 - std::cos(std::abs(rotation) * time)));
                            Eigen::Vector3d target = end_point;
                            target.y() -= stance_width;

                            const Eigen::Matrix3d Raf(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                                      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                                      * Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitZ()));

                            Haf.linear()      = Raf;
                            Haf.translation() = -rASs;
                        }

                        // Move the right foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, true, Haf.matrix(), true, subsumptionId));
                        log("rightstep");
                    } break;


                    case LEFT_STEP: {
                        // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
                        // radians/seconds
                        double rotation = walkcommand.command.z();
                        Eigen::Vector3d translation =
                            Eigen::Vector3d(walkcommand.command.x(), walkcommand.command.y(), 0);
                        Eigen::Affine3d Haf;
                        Eigen::Vector3d rASs;

                        // If there is no rotation to be done, just set the translation to x and y, and set the
                        // rotation to identity
                        if (rotation == 0 || rotation > rotation_limit) {
                            Eigen::Vector3d target = translation * time;
                            target.y() += stance_width;
                            Haf.linear() = Eigen::Matrix3d::Identity();
                            Eigen::Affine3d Hfa;
                            Hfa.linear()      = Eigen::Matrix3d::Identity();
                            Hfa.translation() = target;
                            Haf.translation() = Hfa.inverse().translation();
                        }

                        // If there is rotation, adjust the translation and rotation for this
                        else {
                            //  Multiply by phase time so that we are moving in x metres/second and y metres/second
                            double radius          = translation.norm() / std::abs(rotation);
                            Eigen::Vector3d origin = Eigen::Vector3d(-translation.y(), translation.x(), 0);
                            origin /= rotation;
                            Eigen::Vector3d end_point =
                                rotation > 0 ? (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                                   + (origin * (1 - std::cos(std::abs(rotation) * time)))
                                             : (translation / std::abs(rotation)) * std::sin(std::abs(rotation) * time)
                                                   + (origin * (1 - std::cos(std::abs(rotation) * time)));
                            Eigen::Vector3d target = end_point;
                            target.y() += stance_width / 2;

                            // change target from torso space to support foot space
                            rASs =
                                Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse() * target;

                            const Eigen::Matrix3d Raf(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                                                      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                                                      * Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitZ()));

                            Haf.linear()      = Raf;
                            Haf.translation() = -rASs;
                        }
                        // Move the left foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, false, Haf.matrix(), true, subsumptionId));
                        log("leftstep");
                    } break;
                    default: break;
                }
            });

            emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
                RegisterAction{subsumptionId,
                               "StaticWalk",
                               {std::pair<float, std::set<LimbID>>(10, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<LimbID>&) {},
                               [this](const std::set<ServoID>& servoSet) {}}));
        }
    }  // namespace walk
}  // namespace motion
}  // namespace module
