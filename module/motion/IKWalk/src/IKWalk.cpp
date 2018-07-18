#include "IKWalk.h"

#include <fmt/format.h>

#include "extension/Configuration.h"
#include "extension/Script.h"

#include "message/behaviour/MotionCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/input/Sensors.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/WalkCommand.h"

#include "utility/behaviour/Action.h"
#include "utility/input/LimbID.h"
#include "utility/math/geometry/CubicSpline.h"
#include "utility/math/matrix/Rotation3D.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_expression.h"

namespace module {
namespace motion {

    using extension::Configuration;
    using extension::Script;

    using message::behaviour::MotionCommand;
    using message::behaviour::ServoCommand;
    using message::input::Sensors;
    using message::motion::KinematicsModel;
    using message::motion::ServoTarget;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;

    using utility::behaviour::ActionPriorites;
    using utility::behaviour::RegisterAction;
    using utility::math::matrix::Rotation3D;
    using utility::math::matrix::Transform3D;
    using ServoID = utility::input::ServoID;
    using LimbID  = utility::input::LimbID;
    using utility::support::Expression;


    IKWalk::IKWalk(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))
        , subsumptionId(size_t(this) * size_t(this) - size_t(this))
        , params()
        , kinematicsModel()
        , phase(0.0)
        , dt(1.0 / UPDATE_FREQUENCY)
        , Htf_left()
        , Htf_right()
        , jointGains() {

        on<Configuration>("IKWalk.yaml").then([this](const Configuration& config) {
            // Complete (two legs) walk cycle frequency in Hertz
            params.freq = config["freq"].as<double>();

            // Length of double support phase in phase time (between 0 and 1)
            // 0 is null double support and full single support
            // 1 is full double support and null single support
            params.supportPhaseRatio = config["supportPhaseRatio"].as<Expression>();

            // Lateral offset on default foot position in meters (foot lateral distance)
            // 0 is default
            // > 0 is both feet external offset
            params.footYOffset = config["footYOffset"].as<Expression>();

            // Vertical rise height of each foot in meters (positive)
            params.riseGain = config["riseGain"].as<Expression>();

            // Vertical foot offset from trunk in meters (positive)
            // 0 is in init position
            // > 0 set the robot lower to the ground
            params.trunkZOffset = config["trunkZOffset"].as<Expression>();

            // Lateral trunk oscillation amplitude in meters (positive)
            params.swingGain = config["swingGain"].as<Expression>();

            // Lateral angular oscillation amplitude of swing trunkRoll in radian
            params.swingRollGain = config["swingRollGain"].as<Expression>();

            // Phase shift of lateral trunk oscillation between 0 and 1
            params.swingPhase = config["swingPhase"].as<Expression>();

            // Foot X-Z spline velocities at ground take off and ground landing.
            // Step stands for X and rise stands for Z velocities.
            // Typical values ranges within 0 and 5.
            // >0 for DownVel is having the foot touching the ground with backward velocity.
            // >0 for UpVel is having the foot going back forward with non perpendicular tangent.
            params.stepUpVel   = config["stepUpVel"].as<Expression>();
            params.stepDownVel = config["stepDownVel"].as<Expression>();
            params.riseUpVel   = config["riseUpVel"].as<Expression>();
            params.riseDownVel = config["riseDownVel"].as<Expression>();

            // Time length in phase time where swing lateral oscillation remains on the same side between 0 and 0.5
            params.swingPause = config["swingPause"].as<Expression>();

            // Swing lateral spline velocity (positive).
            // Control the "smoothness" of swing trajectory.
            // Typical values are between 0 and 5.
            params.swingVel = config["swingVel"].as<Expression>();

            // Forward trunk-foot offset with respect to foot in meters
            // >0 moves the trunk forward
            // <0 moves the trunk backward
            params.trunkXOffset = config["trunkXOffset"].as<Expression>();

            // Lateral trunk-foot offset with respect to foot in meters
            // >0 moves the trunk on the left
            // <0 moves the trunk on the right
            params.trunkYOffset = config["trunkYOffset"].as<Expression>();

            // Trunk angular rotation around Y in radian
            // >0 bends the trunk forward
            // <0 bends the trunk backward
            params.trunkPitch = config["trunkPitch"].as<Expression>();

            // Trunk angular rotation around X in radian
            // >0 bends the trunk on the right
            // <0 bends the trunk on the left
            params.trunkRoll = config["trunkRoll"].as<Expression>();

            // Add extra offset on X, Y and Z direction on left and right feet in meters
            // (Can be used for example to implement dynamic kick)
            params.extraLeftX  = config["extraLeftX"].as<Expression>();
            params.extraLeftY  = config["extraLeftY"].as<Expression>();
            params.extraLeftZ  = config["extraLeftZ"].as<Expression>();
            params.extraRightX = config["extraRightX"].as<Expression>();
            params.extraRightY = config["extraRightY"].as<Expression>();
            params.extraRightZ = config["extraRightZ"].as<Expression>();

            // Add extra angular offset on Yaw, Pitch and Roll rotation of left and right foot in radians
            params.extraLeftYaw    = config["extraLeftYaw"].as<Expression>();
            params.extraLeftPitch  = config["extraLeftPitch"].as<Expression>();
            params.extraLeftRoll   = config["extraLeftRoll"].as<Expression>();
            params.extraRightYaw   = config["extraRightYaw"].as<Expression>();
            params.extraRightPitch = config["extraRightPitch"].as<Expression>();
            params.extraRightRoll  = config["extraRightRoll"].as<Expression>();

            // Stop the walk
            params.enabledGain = 0.0;
            params.stepGain    = 0.0;
            params.lateralGain = 0.0;
            params.turnGain    = 0.0;

            auto& gains    = config["gains"];
            float gainArms = gains["arms"].as<Expression>();
            float gainLegs = gains["legs"].as<Expression>();

            for (int i = 0; i < ServoID::NUMBER_OF_SERVOS; ++i) {
                if (int(i) < 6) {
                    jointGains[i] = gainArms;
                }
                else {
                    jointGains[i] = gainLegs;
                }
            }
        });


        emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(
            RegisterAction{subsumptionId,
                           "IKWalk Engine",
                           {std::pair<double, std::set<LimbID>>(100, {LimbID::LEFT_LEG, LimbID::RIGHT_LEG})},
                           [this](const std::set<LimbID>& givenLimbs) {
                               if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
                                   // legs are available, start
                                   updateHandle.enable();
                               }
                           },
                           [this](const std::set<LimbID>& takenLimbs) {
                               if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
                                   // legs are no longer available, reset walking (too late to stop walking)
                                   updateHandle.disable();
                               }
                           },
                           [this](const std::set<ServoID>&) {
                               // nothing
                           }}));


        on<Startup, Trigger<KinematicsModel>>().then([this](const KinematicsModel& kinematics) {
            kinematicsModel = kinematics;

            // Figure out our kinematics for when all leg servos positions are zero
            Sensors sensors;
            for (size_t servo = 0; servo < ServoID::NUMBER_OF_SERVOS; servo++) {
                sensors.servo.emplace_back(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            }

            auto joints =
                utility::motion::kinematics::calculateLegJointPosition(kinematicsModel, sensors, ServoID::L_ANKLE_ROLL);
            Htf_left = joints[ServoID::L_ANKLE_ROLL];

            joints =
                utility::motion::kinematics::calculateLegJointPosition(kinematicsModel, sensors, ServoID::R_ANKLE_ROLL);
            Htf_right = joints[ServoID::R_ANKLE_ROLL];

            // log("Htf_left:", Htf_left);
            // log("Htf_right:", Htf_right);

            // Init Humanoid Model
            // model = HumanoidModel(kinematicsModel.leg.UPPER_LEG_LENGTH,
            //                       kinematicsModel.leg.LOWER_LEG_LENGTH,
            //                       kinematicsModel.leg.FOOT_HEIGHT,
            //                       2.0 * kinematicsModel.leg.HIP_OFFSET_Y);
        });

        /**
         * Compute and return target motor reference positions using given walk parameters at given phase
         * (between 0 and 1).
         * Phase is updated according to frequency parameter and given time step dt.
         * If inverse kinematics fail an error is thrown
         */
        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, Single, Priority::HIGH>().then([this]() {
            // Compute phase for left and right leg
            double phaseLeft  = boundPhase(phase);
            double phaseRight = boundPhase(phase + 0.5);

            // Compute the length of a step (from ground touch to take off) in phase time
            double stepLength = 0.5 * params.supportPhaseRatio + 0.5;

            // Build X foot step spline
            // The foot goes backward between t=0 and t=stepLength and
            // then goes forward. Custom velocity (tangents) are applied at
            // foot take off and landing.
            // During foot backward movement, constant velocity is
            // applied because both feet must have the same velocity
            // during double support phase.
            utility::math::geometry::CubicSpline stepSpline;
            stepSpline.addPoint(0.0, 0.5, -1.0 / stepLength);
            stepSpline.addPoint(stepLength, -0.5, -1.0 / stepLength);
            stepSpline.addPoint(stepLength, -0.5, params.stepUpVel);
            stepSpline.addPoint(1.0, 0.5, -params.stepDownVel);

            // Build Y trunk swing spline.
            // The trunk lateral oscillation goes from right to left, wait a bit (swingPause) on left side
            // then goes to the right and pause as well.
            // Trajectory "smoothness" can be tuned with swingVel updating splines tangents.
            utility::math::geometry::CubicSpline swingSpline;
            swingSpline.addPoint(0.0, -1.0);
            swingSpline.addPoint(params.swingPause * 0.5, -1.0);
            swingSpline.addPoint(params.swingPause * 0.5, -1.0, params.swingVel);
            swingSpline.addPoint(0.5 - params.swingPause * 0.5, 1.0, params.swingVel);
            swingSpline.addPoint(0.5 - params.swingPause * 0.5, 1.0);
            swingSpline.addPoint(0.5 + params.swingPause * 0.5, 1.0);
            swingSpline.addPoint(0.5 + params.swingPause * 0.5, 1.0, -params.swingVel);
            swingSpline.addPoint(1.0 - params.swingPause * 0.5, -1.0, -params.swingVel);
            swingSpline.addPoint(1.0 - params.swingPause * 0.5, -1.0);
            swingSpline.addPoint(1.0, -1.0, 0.0);

            // Build Z foot rise spline.
            // The foot stays on the ground during backward step and then moves up and down.
            // Custom velocities (tangents) can be tuned to achieve specific trajectory at foot take off and landing.
            utility::math::geometry::CubicSpline riseSpline;
            riseSpline.addPoint(0.0, 0.0);
            riseSpline.addPoint(stepLength, 0.0);
            riseSpline.addPoint(stepLength, 0.0, params.riseUpVel);
            riseSpline.addPoint((1.0 + stepLength) * 0.5, 1.0);
            riseSpline.addPoint(1.0, 0.0, -params.riseDownVel);

            // Build Yaw foot turn spline.
            // This is the same as stepSpline but movement occurs only during single support phase as robot
            // degrees of freedom could not achieve rotation during double support phase.
            utility::math::geometry::CubicSpline turnSpline;
            turnSpline.addPoint(0.0, 0.0);
            turnSpline.addPoint(stepLength - 0.5, 0.0);
            turnSpline.addPoint(0.5, 1.0);
            turnSpline.addPoint(stepLength, 1.0);
            turnSpline.addPoint(1.0, 0.0);

            // Compute swing value
            double swingVal =
                params.enabledGain * params.swingGain * swingSpline.posMod(0.5 + phaseLeft + params.swingPhase);

            // Compute feet forward (step) oscillation
            double leftX  = params.enabledGain * params.stepGain * stepSpline.pos(phaseLeft);
            double rightX = params.enabledGain * params.stepGain * stepSpline.pos(phaseRight);

            // Compute feet swing oscillation
            double leftY  = swingVal;
            double rightY = swingVal;

            // Compute feet lateral movement oscillation
            leftY += params.enabledGain * params.lateralGain
                     * (stepSpline.pos(phaseLeft) + 0.5 * utility::math::sign(params.lateralGain));
            rightY += params.enabledGain * params.lateralGain
                      * (stepSpline.pos(phaseRight) - 0.5 * utility::math::sign(params.lateralGain));

            // Set feet lateral offset (feet distance from trunk centre)
            leftY += params.footYOffset;
            rightY += -params.footYOffset;

            // Compute feet vertical (rise) oscillation and offset
            double leftZ  = params.enabledGain * params.riseGain * riseSpline.pos(phaseLeft);
            double rightZ = params.enabledGain * params.riseGain * riseSpline.pos(phaseRight);

            // Set trunk to foot distance height offset
            leftZ += params.trunkZOffset;
            rightZ += params.trunkZOffset;

            // Compute feet rotation (turn) oscillation
            double leftYaw  = params.enabledGain * params.turnGain * turnSpline.pos(phaseLeft);
            double rightYaw = params.enabledGain * params.turnGain * turnSpline.pos(phaseRight);

            // Compute trunk roll angle
            double rollVal =
                params.enabledGain * -params.swingRollGain * swingSpline.posMod(0.5 + phaseLeft + params.swingPhase);

            // Set trunk roll offset
            rollVal += params.trunkRoll;

            // Set feet orientation
            double leftPitch  = params.trunkPitch;
            double leftRoll   = rollVal;
            double rightPitch = params.trunkPitch;
            double rightRoll  = rollVal;

            // Add custom extra foot offset on both feet
            leftX += params.extraLeftX;
            leftY += params.extraLeftY;
            leftZ += params.extraLeftZ;
            leftYaw += params.extraLeftYaw;
            leftPitch += params.extraLeftPitch;
            leftRoll += params.extraLeftRoll;
            rightX += params.extraRightX;
            rightY += params.extraRightY;
            rightZ += params.extraRightZ;
            rightYaw += params.extraRightYaw;
            rightPitch += params.extraRightPitch;
            rightRoll += params.extraRightRoll;

            // Build rotation matrix for trunk pitch and roll
            // orientation
            Eigen::AngleAxisd pitchRot(-params.trunkPitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(-rollVal, Eigen::Vector3d::UnitX());
            Eigen::Matrix3d rotation((pitchRot * rollRot).matrix());

            // Build target vector.
            // Used Euler angles orders is Pitch Roll Yaw because
            // Yaw has to be applied last, after the foot get the good
            // ground orientation. Roll has to be applied after Pitch.
            Eigen::Vector3d posLeft(leftX, leftY, leftZ);
            Eigen::Vector3d angleLeft(leftPitch, leftRoll, leftYaw);
            Eigen::Vector3d posRight(rightX, rightY, rightZ);
            Eigen::Vector3d angleRight(rightPitch, rightRoll, rightYaw);

            // Rotate built feet trajectory to
            // meet asked trunk Pitch and Roll new
            // ground orientation
            posLeft  = rotation * posLeft;
            posRight = rotation * posRight;

            // Apply trunk X-Y offset
            posLeft(0) -= params.trunkXOffset;
            posRight(0) -= params.trunkXOffset;
            posLeft(1) -= params.trunkYOffset;
            posRight(1) -= params.trunkYOffset;

            // In case of trunk Roll rotation, a height (Z)
            // positive offset have to be applied on external foot to
            // set both feet on same level
            double deltaLen = kinematicsModel.leg.HIP_OFFSET_Y * std::tan(rollVal);
            if (rollVal > 0.0) {
                posRight(2) += deltaLen;
            }
            else if (rollVal < 0.0) {
                posLeft(2) -= deltaLen;
            }

            // TODO In case of oscillating trunkRoll or swingRoll enabled XXX
            // TODO feet get an unwanted lateral oscillation XXX

            double legsLength = kinematicsModel.leg.UPPER_LEG_LENGTH + kinematicsModel.leg.LOWER_LEG_LENGTH
                                + kinematicsModel.leg.FOOT_HEIGHT;
            // Trunk X and Y offset is applied to compensate
            // Pitch and Roll rotation. It is better for tuning if
            // trunk pitch or roll rotation do not apply offset on
            // trunk position.
            posLeft.x() += legsLength * std::tan(params.trunkPitch) - kinematicsModel.leg.TOE_LENGTH;
            posRight.x() += legsLength * std::tan(params.trunkPitch) - kinematicsModel.leg.TOE_LENGTH;
            posLeft.y() -= legsLength * std::tan(rollVal);
            posRight.y() -= legsLength * std::tan(rollVal);
            posLeft.z() -= legsLength;
            posRight.z() -= legsLength;

            // Transform left and right foot positions into torso space
            posLeft  = convert<double, 3>(Htf_left * convert<double, 3>(posLeft));
            posRight = convert<double, 3>(Htf_right * convert<double, 3>(posRight));
            log(posLeft);
            log(posRight);

            // Convert left leg into a homogeneous transform
            Eigen::AngleAxisd yaw(angleLeft.z(), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitch(angleLeft.x(), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd roll(angleLeft.y(), Eigen::Vector3d::UnitX());

            Transform3D left(Rotation3D(convert<double, 3, 3>((yaw * roll * pitch).matrix())),
                             convert<double, 3>(posLeft));
            bool leftValid = utility::motion::kinematics::legPoseValid(kinematicsModel, left, LimbID::LEFT_LEG);

            // Convert right leg into a homogeneous transform
            yaw   = Eigen::AngleAxisd(angleRight.z(), Eigen::Vector3d::UnitZ());
            pitch = Eigen::AngleAxisd(angleRight.x(), Eigen::Vector3d::UnitY());
            roll  = Eigen::AngleAxisd(angleRight.y(), Eigen::Vector3d::UnitX());

            Transform3D right(Rotation3D(convert<double, 3, 3>((yaw * roll * pitch).matrix())),
                              convert<double, 3>(posRight));
            bool rightValid = utility::motion::kinematics::legPoseValid(kinematicsModel, right, LimbID::RIGHT_LEG);

            if (leftValid && rightValid) {
                // Increment given phase
                // Cycling between 0 and 1
                phase = boundPhase(phase + dt * params.freq);

                // Emit servo waypoints
                NUClear::clock::time_point time =
                    NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

                auto joints = utility::motion::kinematics::calculateLegJoints(kinematicsModel, left, right);

                std::unique_ptr<std::vector<ServoCommand>> waypoints = std::make_unique<std::vector<ServoCommand>>();
                waypoints->reserve(12);

                for (const auto& joint : joints) {
                    // TODO: support separate gains for each leg
                    waypoints->push_back(
                        {subsumptionId, time, joint.first, float(joint.second), jointGains[joint.first], 100});
                }
                emit(std::move(waypoints));
            }

            else if (!leftValid) {
                log<NUClear::FATAL>("Invalid walk parameters. Left leg pose is not valid.");
                log<NUClear::FATAL>(left);
                log<NUClear::FATAL>(angleLeft);
                log<NUClear::FATAL>(posLeft);
            }

            else {
                log<NUClear::FATAL>("Invalid walk parameters. Right leg pose is not valid.");
                log<NUClear::FATAL>(right);
                log<NUClear::FATAL>(angleRight);
                log<NUClear::FATAL>(posRight);
            }

            if (params.enabledGain == 0.0) {
                updateHandle.disable();
            }

            // Run inverse invert kinematics on both legs
            // using Pitch-Roll-Yaw convention
            // IKWalkOutputs outputs;
            // bool successLeft  = model.legIK(posLeft, angleLeft, module::motion::EulerPitchRollYaw, true, outputs);
            // bool successRight = model.legIK(posRight, angleRight, module::motion::EulerPitchRollYaw, false, outputs);

            // // Check inverse kinematics success
            // if (successLeft && successRight) {
            //     // Increment given phase
            //     // Cycling between 0 and 1
            //     phase = boundPhase(phase + dt * params.freq);

            //     // Emit servo waypoints
            //     NUClear::clock::time_point time =
            //         NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

            //     std::unique_ptr<std::vector<ServoCommand>> waypoints = std::make_unique<std::vector<ServoCommand>>();
            //     waypoints->reserve(12);

            //     // TODO: support separate gains for each leg
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::L_HIP_YAW,
            //                           float(outputs.left_hip_yaw),
            //                           jointGains[ServoID::L_HIP_YAW],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_HIP_YAW,
            //                           float(outputs.right_hip_yaw),
            //                           jointGains[ServoID::R_HIP_YAW],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::L_HIP_ROLL,
            //                           float(outputs.left_hip_roll),
            //                           jointGains[ServoID::L_HIP_ROLL],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_HIP_ROLL,
            //                           float(outputs.right_hip_roll),
            //                           jointGains[ServoID::R_HIP_ROLL],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::L_HIP_PITCH,
            //                           float(outputs.left_hip_pitch),
            //                           jointGains[ServoID::L_HIP_PITCH],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_HIP_PITCH,
            //                           float(outputs.right_hip_pitch),
            //                           jointGains[ServoID::R_HIP_PITCH],
            //                           100});
            //     waypoints->push_back(
            //         {subsumptionId, time, ServoID::L_KNEE, float(outputs.left_knee), jointGains[ServoID::L_KNEE],
            //         100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_KNEE,
            //                           float(outputs.right_knee),
            //                           jointGains[ServoID::R_KNEE],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::L_ANKLE_PITCH,
            //                           float(outputs.left_ankle_pitch),
            //                           jointGains[ServoID::L_ANKLE_PITCH],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_ANKLE_PITCH,
            //                           float(outputs.right_ankle_pitch),
            //                           jointGains[ServoID::R_ANKLE_PITCH],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::L_ANKLE_ROLL,
            //                           float(outputs.left_ankle_roll),
            //                           jointGains[ServoID::L_ANKLE_ROLL],
            //                           100});
            //     waypoints->push_back({subsumptionId,
            //                           time,
            //                           ServoID::R_ANKLE_ROLL,
            //                           float(outputs.right_ankle_roll),
            //                           jointGains[ServoID::R_ANKLE_ROLL],
            //                           100});

            //     emit(std::move(waypoints));
            // }
            // else {
            //     log<NUClear::FATAL>("Invalid walk parameters. Servo waypoints are not finite.");
            //     log<NUClear::FATAL>(fmt::format("failure: {}, {}.", successLeft, successRight));
            //     log<NUClear::FATAL>(fmt::format("hip_yaw: {}, {}\n", outputs.left_hip_yaw, outputs.right_hip_yaw));
            //     log<NUClear::FATAL>(fmt::format("hip_roll: {}, {}\n", outputs.left_hip_roll,
            //     outputs.right_hip_roll)); log<NUClear::FATAL>(
            //         fmt::format("hip_pitch: {}, {}\n", outputs.left_hip_pitch, outputs.right_hip_pitch));
            //     log<NUClear::FATAL>(fmt::format("knee: {}, {}\n", outputs.left_knee, outputs.right_knee));
            //     log<NUClear::FATAL>(
            //         fmt::format("ankle_pitch: {}, {}\n", outputs.left_ankle_pitch, outputs.right_ankle_pitch));
            //     log<NUClear::FATAL>(
            //         fmt::format("ankle_roll: {}, {}\n", outputs.left_ankle_roll, outputs.right_ankle_roll));
            // }
        });

        on<Trigger<MotionCommand>>().then([this](const MotionCommand& motionCommand) {
            if (motionCommand.type == MotionCommand::Type::Value::DirectCommand) {
                updateHandle.enable();
                if ((motionCommand.walkCommand.x() == 0) && (motionCommand.walkCommand.y() == 0)
                    && (motionCommand.walkCommand.z() == 0)) {
                    // Walk in place
                    params.enabledGain = 1.0;
                    params.stepGain    = 0.0;
                    params.lateralGain = 0.0;
                    params.turnGain    = 0.0;
                }

                else {
                    params.enabledGain = 1.0;
                    params.stepGain    = motionCommand.walkCommand.x();  // Forward/backward
                    params.lateralGain = motionCommand.walkCommand.y();  // Lateral left/right
                    params.turnGain    = motionCommand.walkCommand.z();  // Turn left/right
                }
            }

            else if (motionCommand.type == MotionCommand::Type::Value::StandStill) {
                // Note that in practice params.stepGain = 0.0 does not make the robot walk on place.
                // Some offset step trim have to be tuned to really find the robot "neutral".

                // The walk is stopped
                params.enabledGain = 0.0;
                params.stepGain    = 0.0;
                params.lateralGain = 0.0;
                params.turnGain    = 0.0;
            }
        });

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            if ((walkCommand.command.x() == 0) && (walkCommand.command.y() == 0) && (walkCommand.command.z() == 0)) {
                // Walk in place
                params.enabledGain = 1.0;
                params.stepGain    = 0.0;
                params.lateralGain = 0.0;
                params.turnGain    = 0.0;
            }

            else {
                params.enabledGain = 1.0;
                params.stepGain    = walkCommand.command.x();  // Forward/backward
                params.lateralGain = walkCommand.command.y();  // Lateral left/right
                params.turnGain    = walkCommand.command.z();  // Turn left/right
                updateHandle.enable();
            }
        });

        on<Trigger<StopCommand>>().then([this] {
            // Note that in practice params.stepGain = 0.0 does not make the robot walk on place.
            // Some offset step trim have to be tuned to really find the robot "neutral".

            // The walk is stopped
            params.enabledGain = 0.0;
            params.stepGain    = 0.0;
            params.lateralGain = 0.0;
            params.turnGain    = 0.0;
        });
    }  // namespace motion
}  // namespace motion
}  // namespace module
