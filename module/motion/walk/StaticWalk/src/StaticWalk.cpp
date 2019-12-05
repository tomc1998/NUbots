#include "StaticWalk.h"

#include <Eigen/Geometry>
#include <vector>
#include "extension/Configuration.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/TorsoTarget.h"
#include "message/motion/WalkCommand.h"
#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/input/ServoID.h"
#include "utility/math/comparison.h"
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
        using message::motion::WalkCommand;
        using utility::behaviour::RegisterAction;
        using utility::input::LimbID;
        using utility::input::ServoID;
        using utility::math::almost_equal;
        using utility::math::matrix::Transform3D;
        using utility::motion::kinematics::calculateGroundSpace;
        using utility::motion::kinematics::calculateLegJoints;
        using utility::motion::kinematics::legPoseValid;
        using utility::nusight::graph;
        using utility::support::Expression;

        // Retrieves the target for the lean based on support foot, ground foot, and COM location
        Eigen::Affine3d StaticWalk::getLeanTarget(const Eigen::Affine3d Hts,
                                                  double y_offset_local,
                                                  const Eigen::Affine3d Htg,
                                                  const Eigen::Vector3d rCTt) {
            // This will be the returned matrix, ground to torso target
            Eigen::Affine3d Ht_tg;

            // Need to find how much to move the torso to so that the COM is over the foot
            // Get the vector from torso to support in torso space
            // Then get the vector from CoM to support foot in torso space
            Eigen::Vector3d rSTt(Hts.translation());
            Eigen::Vector3d rSCt(rSTt - rCTt);

            // Going from support foot to the torso, we add on the CoM to torso vector
            // If the CoM is on the torso, this will move the torso target to above the foot
            // If CoM is closer to the foot than the torso, the torso will not move the whole way to the support foot
            // If CoM is further away than the torso to the foot, the torso will move past the foot to land the CoM on
            // the foot
            Eigen::Vector3d rT_tSt = Eigen::Vector3d(-rSTt + rSCt);

            // Fix up the vector using configuration variables. We want to set the height to our defined torso height
            // that we want to robot to move to. The offsets set the CoM inside the support polygon rather than on the
            // edge, since the support foot is measured from the ankle servo which is at the back of the foot.
            rT_tSt.z() = torso_height;
            // rT_tSt.x() += x_offset;
            // rT_tSt.y() += y_offset;

            Eigen::Vector3d rT_tGt = (Htg.rotation() * (Htg.inverse() * Hts).translation()) + rT_tSt;

            // Set the rotation relative to the ground as the identity matrix,
            // since we want the torso to rotate into ground space or stay in ground space
            Ht_tg.linear()      = Eigen::Matrix3d::Identity();
            Ht_tg.translation() = Eigen::Vector3d(0, 0, -torso_height);

            return Ht_tg;
        }

        // Find the foot target for the step
        Eigen::Affine3d StaticWalk::getFootTarget(const enum State state, const Eigen::Vector3d walkcommand) {
            // walkcommand is (x,y,theta) where x,y is velocity in m/s and theta is angle in
            // radians/seconds
            // log("rightstep2");

            // Clamp the rotation to rotation limit
            double rotation = walkcommand.z() > rotation_limit ? rotation_limit : walkcommand.z();

            // Set translation vector, z component is 0
            Eigen::Vector3d translation = Eigen::Vector3d(walkcommand.x(), walkcommand.y(), 0);

            // Foot to foot target matrix
            Eigen::Affine3d Haf;
            Eigen::Vector3d rASs;

            // TODO: write stop function and proper check
            // If there is no rotation to be done, just set the translation to x and y, and set the
            // rotation to identity
            // rotation check is to stop the robot from turning outside acceptable parameters
            if (rotation == 0) {
                Eigen::Vector3d target = translation * time;
                target.y()             = state == LEFT_STEP ? target.y() + stance_width : target.y() - stance_width;

                Eigen::Affine3d Hfa;
                Hfa.linear()      = Eigen::Matrix3d::Identity();
                Hfa.translation() = target;
                Haf               = Hfa.inverse();
            }

            // If there is rotation, adjust the translation and rotation for this
            else {
                //  Multiply by phase time so that we are moving in x metres/second and y metres/second
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

            return Haf;
        }

        StaticWalk::StaticWalk(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment)), subsumptionId(size_t(this) * size_t(this) - size_t(this)) {
            on<Configuration>("StaticWalk.yaml").then([this](const Configuration& config) {
                // Set initial conditions
                start_phase = NUClear::clock::now();
                state       = INITIAL;
                start_right_lean =
                    config["start_right_lean"].as<bool>();  // TODO: dynamically determine which foot to start on

                // Use configuration here from file StaticWalk.yaml
                torso_height   = config["torso_height"].as<double>();
                stance_width   = config["stance_width"].as<double>();
                phase_time     = std::chrono::milliseconds(config["phase_time"].as<int>());
                y_offset       = config["y_offset"].as<double>();
                x_offset       = config["x_offset"].as<double>();
                rotation_limit = config["rotation_limit"].as<Expression>();

                // Set test parameters from config
                double x_speed = config["test"]["x_speed"].as<double>();
                double y_speed = config["test"]["y_speed"].as<double>();
                double angle   = config["test"]["angle"].as<double>();

                // Multiply by 2 so that the lean states are accounted for
                time = std::chrono::duration_cast<std::chrono::duration<double>>(phase_time).count() * 2;

                // Emit the walk command
                emit(std::make_unique<WalkCommand>(subsumptionId, Eigen::Vector3d(x_speed, y_speed, angle)));
            });

            on<Trigger<Sensors>, With<WalkCommand>>().then([this](const Sensors& sensors,
                                                                  const WalkCommand& walkcommand) {
                // INITIAL state occurs only as the first state in the walk to set the matrix Hff_s
                if (state == INITIAL) {
                    // Set the state based on the config
                    state = start_right_lean ? RIGHT_LEAN : LEFT_LEAN;

                    // Get swing foot in ground space
                    Eigen::Affine3d Hts = state == LEFT_LEAN
                                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL])
                                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]);

                    Eigen::Affine3d Htg(calculateGroundSpace(Hts, Eigen::Affine3d(sensors.Htw).inverse()));

                    Hwg = state == LEFT_LEAN
                              ? Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse() * Htg
                              : Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse() * Htg;

                    // Force the foot to be in ground space. TODO: check this
                    Hwg.linear() = Eigen::Matrix3d::Identity();
                }

                // When the time is over for this phase, begin the next phase
                if (NUClear::clock::now() > start_phase + phase_time) {
                    // Reset the start phase time for the new phase
                    start_phase = NUClear::clock::now();

                    // Change the state of the walk based on what the previous state was
                    switch (state) {
                        case LEFT_LEAN: state = RIGHT_STEP; break;
                        case RIGHT_STEP: {
                            // Store where support is relative to swing in ground space
                            Eigen::Affine3d Htg(
                                calculateGroundSpace(Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]),
                                                     Eigen::Affine3d(sensors.Htw).inverse()));
                            Hwg = Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]).inverse() * Htg;

                            // Force the foot into ground space. TODO: check this
                            Hwg.linear() = Eigen::Matrix3d::Identity();

                            state = RIGHT_LEAN;
                        } break;
                        case RIGHT_LEAN: state = LEFT_STEP; break;
                        case LEFT_STEP: {
                            // Store where support is relative to swing in ground space
                            Eigen::Affine3d Htg(
                                calculateGroundSpace(Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]),
                                                     Eigen::Affine3d(sensors.Htw).inverse()));
                            Hwg = Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]).inverse() * Htg;

                            // Force the foot into ground space. TODO: check this
                            Hwg.linear() = Eigen::Matrix3d::Identity();

                            state = LEFT_LEAN;
                        } break;
                        default: break;
                    }
                }

                // Put our COM over the correct foot or move foot to target, based on which state we are in
                switch (state) {
                    case LEFT_LEAN: {
                        // Move the torso over the left foot
                        emit(std::make_unique<TorsoTarget>(
                            start_phase + phase_time,
                            false,
                            getLeanTarget(
                                Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]),
                                -y_offset,
                                calculateGroundSpace(Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_ANKLE_ROLL]),
                                                     Eigen::Affine3d(sensors.Htw).inverse()),
                                Eigen::Vector3d(sensors.centre_of_mass.x(), sensors.centre_of_mass.y(), 0))
                                .matrix(),
                            subsumptionId));

                        // Keep the swing foot in place relative to support, with ground rotation
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, true, Hwg.matrix(), false, subsumptionId));
                    } break;
                    case RIGHT_LEAN: {
                        // Move the torso over the left foot
                        emit(std::make_unique<TorsoTarget>(
                            start_phase + phase_time,
                            true,
                            getLeanTarget(
                                Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]),
                                y_offset,
                                calculateGroundSpace(Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_ANKLE_ROLL]),
                                                     Eigen::Affine3d(sensors.Htw).inverse()),
                                Eigen::Vector3d(sensors.centre_of_mass.x(), sensors.centre_of_mass.y(), 0))
                                .matrix(),
                            subsumptionId));

                        // Maintain left foot position while the torso moves over the right foot
                        emit(std::make_unique<FootTarget>(
                            start_phase + phase_time, false, Hwg.matrix(), false, subsumptionId));
                    } break;


                    case RIGHT_STEP: {
                        // Move the right foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(start_phase + phase_time,
                                                          true,
                                                          getFootTarget(RIGHT_STEP,
                                                                        Eigen::Vector3d(walkcommand.command.x(),
                                                                                        walkcommand.command.y(),
                                                                                        walkcommand.command.z()))
                                                              .matrix(),
                                                          true,
                                                          subsumptionId));
                        //log("rightstep");
                    } break;

                    case LEFT_STEP: {
                        // Move the left foot to the location specified by the walkcommand
                        emit(std::make_unique<FootTarget>(start_phase + phase_time,
                                                          false,
                                                          getFootTarget(LEFT_STEP,
                                                                        Eigen::Vector3d(walkcommand.command.x(),
                                                                                        walkcommand.command.y(),
                                                                                        walkcommand.command.z()))
                                                              .matrix(),
                                                          true,
                                                          subsumptionId));
                        //log("leftstep");
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
