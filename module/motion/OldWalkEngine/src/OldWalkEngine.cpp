/*
 * This file is part of OldWalkEngine.
 *
 * OldWalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OldWalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OldWalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "OldWalkEngine.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "extension/Configuration.h"
#include "extension/Script.h"
#include "message/behaviour/FixedWalkCommand.h"
#include "message/behaviour/ServoCommand.h"
#include "message/motion/KinematicsModel.h"
#include "message/motion/ServoTarget.h"
#include "message/motion/WalkCommand.h"
#include "message/support/SaveConfiguration.h"
#include "utility/math/angle.h"
#include "utility/math/comparison.h"
#include "utility/motion/Balance.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/InverseKinematics.h"
#include "utility/nusight/NUhelpers.h"
#include "utility/support/yaml_expression.h"
#include "utility/math/matrix/transform.h"

namespace module {
namespace motion {

    using extension::Configuration;
    using extension::Script;

    using message::behaviour::ServoCommand;
    using message::behaviour::WalkConfigSaved;
    using message::behaviour::WalkOptimiserCommand;
    using message::input::Sensors;
    using message::motion::DisableWalkEngineCommand;
    using message::motion::EnableWalkEngineCommand;
    using message::motion::KinematicsModel;
    using message::motion::ServoTarget;
    using message::motion::StopCommand;
    using message::motion::WalkCommand;
    using message::motion::WalkStopped;
    using message::support::SaveConfiguration;

    using ServoID = utility::input::ServoID;
    using LimbID  = utility::input::LimbID;
    using utility::math::clamp;
    using utility::math::angle::normalizeAngle;
    using utility::motion::kinematics::calculateLegJoints;
    using utility::motion::kinematics::calculateLegJointsTeamDarwin;
    using utility::nusight::graph;
    using utility::support::Expression;
    using utility::math::transform::worldToLocal;
    using utility::math::transform::localToWorld;
    using utility::math::transform::interpolate;
    using utility::math::transform::angle;
    using utility::math::transform::rotateZLocal;
    using utility::math::transform::rotateX;
    using utility::math::transform::rotateY;
    using utility::math::transform::rotateZ;
    using utility::math::transform::twoD_to_threeD;


    OldWalkEngine::OldWalkEngine(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment))

        , updateHandle()
        , state()
        , startFromStep(false)
        , beginStepTime(0.0)
        , initialStep(0)
        , uTorso()
        , uTorsoSource()
        , uTorsoDestination()
        , uLeftFoot()
        , uLeftFootSource()
        , uLeftFootDestination()
        , uRightFoot()
        , uRightFootSource()
        , uRightFootDestination()
        , uSupport()
        , velocityCurrent()
        , velocityCommand()
        , velocityDifference()
        , zmpCoefficients(Eigen::Vector4d::Zero())
        , zmpParams(Eigen::Vector4d::Zero())
        , swingLeg()
        , lastFootGoalRotation()
        , footGoalErrorSum()
        , stanceLimitY2(0.0)
        , stepLimits(Eigen::Matrix<double, 3, 2>::Zero())
        , velocityLimits(Eigen::Matrix<double, 3, 2>::Zero())
        , accelerationLimits(Eigen::Vector3d::Zero())
        , accelerationLimitsHigh(Eigen::Vector3d::Zero())
        , velocityHigh(0.0)
        , accelerationTurningFactor(0.0)
        , bodyHeight(0.0)
        , bodyTilt(0.0)
        , gainArms(0.0f)
        , gainLegs(0.0f)
        , stepTime(0.0)
        , zmpTime(0.0)
        , stepHeight(0.0)
        , step_height_slow_fraction(0.0f)
        , step_height_fast_fraction(0.0f)
        , phase1Single(0.0)
        , phase2Single(0.0)
        , footOffset(Eigen::Vector2d::Zero())
        , legYaw(0.0)
        , uLRFootOffset()
        , qLArmStart(Eigen::Vector3d::Zero())
        , qLArmEnd(Eigen::Vector3d::Zero())
        , qRArmStart(Eigen::Vector3d::Zero())
        , qRArmEnd(Eigen::Vector3d::Zero())
        , balanceEnabled(0.0)
        , balanceAmplitude(0.0)
        , balanceWeight(0.0)
        , balanceOffset(0.0)
        , balancePGain(0.0)
        , balanceIGain(0.0)
        , balanceDGain(0.0)
        , lastVeloctiyUpdateTime()
        , jointGains()
        , servoControlPGains()
        , balancer()
        , pushTime()
        , kinematicsModel()
        , hipRollCompensation(0.0)
        , STAND_SCRIPT_DURATION(0.0)
        , generateStandScriptReaction() {
        // , subsumptionId(size_t(this) * size_t(this) - size_t(this)) {

        // emit<Scope::INITIALIZE>(std::make_unique<RegisterAction>(RegisterAction {
        //     subsumptionId,
        //     "Walk Engine",
        //     {
        //         std::pair<double, std::set<LimbID>>(0, {LimbID::LEFT_LEG,
        //         LimbID::RIGHT_LEG}), std::pair<double, std::set<LimbID>>(0,
        //         {LimbID::LEFT_ARM, LimbID::RIGHT_ARM}),
        //     },
        //     [this] (const std::set<LimbID>& givenLimbs) {
        //         if (givenLimbs.find(LimbID::LEFT_LEG) != givenLimbs.end()) {
        //             // legs are available, start
        //             stanceReset(); // reset stance as we don't know where our limbs
        //             are interrupted = false; updateHandle.enable();
        //         }
        //     },
        //     [this] (const std::set<LimbID>& takenLimbs) {
        //         if (takenLimbs.find(LimbID::LEFT_LEG) != takenLimbs.end()) {
        //             // legs are no longer available, reset walking (too late to
        //             stop walking) updateHandle.disable(); interrupted = true;
        //         }
        //     },
        //     [this] (const std::set<ServoID>&) {
        //         // nothing
        //     }
        // }));

        on<Startup, Trigger<KinematicsModel>>().then("Update Kin Model",
                                                     [this](const KinematicsModel& model) { kinematicsModel = model; });

        on<Trigger<EnableWalkEngineCommand>>().then([this](const EnableWalkEngineCommand& command) {
            subsumptionId = command.subsumptionId;

            stanceReset();  // Reset stance as we don't know where our limbs are.
            updateHandle.enable();
        });

        on<Trigger<DisableWalkEngineCommand>>().then([this] {
            // Nobody needs the walk engine, so we stop updating it.
            updateHandle.disable();

            // TODO: Also disable the other walk command reactions?
        });

        updateHandle = on<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>, With<Sensors>, Single, Priority::HIGH>()
                           .then([this](const Sensors& sensors) { update(sensors); })
                           .disable();

        on<Trigger<WalkCommand>>().then([this](const WalkCommand& walkCommand) {
            Eigen::Affine2d velocity;
            velocity.translation() = Eigen::Vector2d({walkCommand.command[0], walkCommand.command[1]});
            velocity.linear() = Eigen::Rotation2Dd(walkCommand.command[2]).toRotationMatrix();
            if (velocity.translation()[0] == 0 && velocity.translation()[1] == 0 && angle(velocity) == 0) {
                requestStop();
            }

            velocity.translation()[0] *= velocity.translation()[0] > 0 ? velocityLimits(0, 1) : -velocityLimits(0, 0);
            velocity.translation()[1] *= velocity.translation()[1] > 0 ? velocityLimits(1, 1) : -velocityLimits(1, 0);
            
            // Rotate velocity transform RHS radians
            double velocityRot = angle(velocity) > 0 ? velocityLimits(2, 1) : -velocityLimits(2, 0);
            velocity.linear() = velocity.linear() * Eigen::Rotation2Dd(velocityRot).toRotationMatrix();

            setVelocity(velocity);
            lastVeloctiyUpdateTime = NUClear::clock::now();
            start();
        });

        on<Trigger<StopCommand>>().then([this] {
            // TODO: This sets STOP_REQUEST, which appears not to be used anywhere.
            // If this is the case, we should delete or rethink the WalkStopCommand.
            requestStop();
        });

        on<Configuration>("OldWalkEngine.yaml").then([this](const Configuration& config) { configure(config.config); });

        // TODO: finish push detection and compensation
        // pushTime = NUClear::clock::now();
        // on<Trigger<PushDetection>, With<Configuration>>().then([this](const
        // PushDetection& pd, const Configuration& config) {
        //     balanceEnabled = true;
        //     // balanceAmplitude = balance["amplitude"].as<Expression>();
        //     // balanceWeight = balance["weight"].as<Expression>();
        //     // balanceOffset = balance["offset"].as<Expression>();

        //     balancer.configure(config["walk_cycle"]["balance"]["push_recovery"]);
        //     pushTime = NUClear::clock::now();

        //     // configure(config.config);
        // });

        // on<
        //     Every<10, std::chrono::milliseconds>>(
        //     With<Configuration<WalkEngine>>
        // >().then([this](const Configuration& config) {
        //     [this](const WalkOptimiserCommand& command) {
        //     if ((NUClear::clock::now() - pushTime) >
        //     std::chrono::milliseconds(config["walk_cycle"]["balance"]["balance_time"].as<int>))
        //     {
        //         balancer.configure(config["walk_cycle"]["balance"]);
        //     }
        // });

        on<Trigger<WalkOptimiserCommand>>().then([this](const WalkOptimiserCommand& command) {
            configure(YAML::Load(command.walkConfig));
            emit(std::make_unique<WalkConfigSaved>());
        });

        generateStandScriptReaction = on<Trigger<Sensors>, Single>().then([this](const Sensors& /*sensors*/) {
            generateStandScriptReaction.disable();
            // generateAndSaveStandScript(sensors);
            // state = State::LAST_STEP;
            // start();
        });

        reset();
    }

    void OldWalkEngine::configure(const YAML::Node& config) {
        use_com      = config["use_com"].as<bool>();
        auto& stance = config["stance"];
        bodyHeight   = stance["body_height"].as<Expression>();
        bodyTilt     = stance["body_tilt"].as<Expression>();
        qLArmStart   = stance["arms"]["left"]["start"].as<Expression>();
        qLArmEnd     = stance["arms"]["left"]["end"].as<Expression>();
        qRArmStart   = stance["arms"]["right"]["start"].as<Expression>();
        qRArmEnd     = stance["arms"]["right"]["end"].as<Expression>();
        footOffset   = stance["foot_offset"].as<Expression>();
        // gToe/heel overlap checking values
        stanceLimitY2 = stance["limit_margin_y"].as<Expression>();
        stanceLimitY2 = kinematicsModel.leg.LENGTH_BETWEEN_LEGS - stanceLimitY2;

        auto& gains = stance["gains"];
        gainArms    = gains["arms"].as<Expression>();
        gainLegs    = gains["legs"].as<Expression>();

        for (int i = 0; i < ServoID::NUMBER_OF_SERVOS; ++i) {
            if (int(i) < 6) {
                jointGains[i] = gainArms;
            }
            else {
                jointGains[i] = gainLegs;
            }
        }

        auto& walkCycle     = config["walk_cycle"];
        stepTime            = walkCycle["step_time"].as<Expression>();
        zmpTime             = walkCycle["zmp_time"].as<Expression>();
        hipRollCompensation = walkCycle["hip_roll_compensation"].as<Expression>();
        stepHeight          = walkCycle["step"]["height"].as<Expression>();
        stepLimits          = walkCycle["step"]["limits"].as<Expression>();
        legYaw              = walkCycle["step"]["leg_yaw"].as<Expression>();
        ankleRollComp       = walkCycle["step"]["compensation"]["roll_coef"].as<Expression>();
        ankleRollLimit      = walkCycle["step"]["compensation"]["roll_limit"].as<Expression>();
        anklePitchComp      = walkCycle["step"]["compensation"]["pitch_coef"].as<Expression>();
        anklePitchLimit     = walkCycle["step"]["compensation"]["pitch_limit"].as<Expression>();

        step_height_slow_fraction = walkCycle["step"]["height_slow_fraction"].as<float>();
        step_height_fast_fraction = walkCycle["step"]["height_fast_fraction"].as<float>();

        auto& velocity = walkCycle["velocity"];
        velocityLimits = velocity["limits"].as<Expression>();
        velocityHigh   = velocity["high_speed"].as<Expression>();

        auto& acceleration        = walkCycle["acceleration"];
        accelerationLimits        = acceleration["limits"].as<Expression>();
        accelerationLimitsHigh    = acceleration["limits_high"].as<Expression>();
        accelerationTurningFactor = acceleration["turning_factor"].as<Expression>();

        phase1Single = walkCycle["single_support_phase"]["start"].as<Expression>();
        phase2Single = walkCycle["single_support_phase"]["end"].as<Expression>();

        auto& balance  = walkCycle["balance"];
        balanceEnabled = balance["enabled"].as<bool>();
        // balanceAmplitude = balance["amplitude"].as<Expression>();
        // balanceWeight = balance["weight"].as<Expression>();
        // balanceOffset = balance["offset"].as<Expression>();

        balancer.configure(balance);

        for (auto& gain : balance["servo_gains"]) {
            float p = gain["p"].as<Expression>();
            ServoID sr(gain["id"].as<std::string>(), utility::input::ServoSide::RIGHT);
            ServoID sl(gain["id"].as<std::string>(), utility::input::ServoSide::LEFT);
            servoControlPGains[sr] = p;
            servoControlPGains[sl] = p;
        }
        /* TODO
        // gCompensation parameters
        toeTipCompensation = config["toeTipCompensation"].as<Expression>();
        ankleMod = {-toeTipCompensation, 0};

        // gGyro stabilization parameters
        ankleImuParamX = config["ankleImuParamX"].as<arma::vec>();
        ankleImuParamY = config["ankleImuParamY"].as<arma::vec>();
        kneeImuParamX = config["kneeImuParamX"].as<arma::vec>();
        hipImuParamY = config["hipImuParamY"].as<arma::vec>();
        armImuParamX = config["armImuParamX"].as<arma::vec>();
        armImuParamY = config["armImuParamY"].as<arma::vec>();

        // gSupport bias parameters to reduce backlash-based instability
        velFastForward = config["velFastForward"].as<Expression>();
        velFastTurn = config["velFastTurn"].as<Expression>();
        supportFront = config["supportFront"].as<Expression>();
        supportFront2 = config["supportFront2"].as<Expression>();
        supportBack = config["supportBack"].as<Expression>();
        supportSideX = config["supportSideX"].as<Expression>();
        supportSideY = config["supportSideY"].as<Expression>();
        supportTurn = config["supportTurn"].as<Expression>();

        */
        STAND_SCRIPT_DURATION = config["STAND_SCRIPT_DURATION"].as<Expression>();
    }

    void OldWalkEngine::generateAndSaveStandScript(const Sensors& sensors) {
        reset();
        stanceReset();
        auto waypoints = updateStillWayPoints(sensors);

        Script standScript;
        Script::Frame frame;
        frame.duration = std::chrono::milliseconds(int(round(1000 * STAND_SCRIPT_DURATION)));
        for (auto& waypoint : *waypoints) {
            frame.targets.push_back(
                Script::Frame::Target({waypoint.id, waypoint.position, std::max(waypoint.gain, 60.0f), 100}));
        }
        standScript.frames.push_back(frame);
        standScript.save("Stand.yaml");

        // Try update(); ?
        reset();
        stanceReset();
    }

    void OldWalkEngine::stanceReset() {
        // standup/sitdown/falldown handling
        if (startFromStep) {
            uLeftFoot.matrix()  = Eigen::Matrix3d::Identity();
            uRightFoot.matrix() = Eigen::Matrix3d::Identity();
            uTorso.matrix()     = Eigen::Matrix3d::Identity();

            // start walking asap
            initialStep = 1;
        }
        else {
            // stance resetted
            Eigen::Affine2d footOffsetTemp;
            footOffsetTemp.linear().matrix() = Eigen::Matrix2d::Identity();
            
            // Each foot has slightly different .translation()[1] assigned
            footOffsetTemp.translation() = Eigen::Vector2d({footOffset[0], kinematicsModel.leg.HIP_OFFSET_Y - footOffset[1]});
            uLeftFoot   = localToWorld(uTorso, footOffsetTemp);

            footOffsetTemp.translation() = Eigen::Vector2d({footOffset[0], -kinematicsModel.leg.HIP_OFFSET_Y + footOffset[1]});
            uRightFoot  = localToWorld(uTorso,  footOffsetTemp);

            initialStep = 2;
        }

        swingLeg = swingLegInitial;

        uLeftFootSource      = uLeftFoot;
        uLeftFootDestination = uLeftFoot;

        uRightFootSource      = uRightFoot;
        uRightFootDestination = uRightFoot;

        uSupport      = uTorso;
        beginStepTime = getTime();
        uLRFootOffset.translation() = Eigen::Vector2d({0, kinematicsModel.leg.HIP_OFFSET_Y - footOffset[1]});
        uLRFootOffset.linear().matrix() = Eigen::Matrix2d::Identity();
        startFromStep = false;
        calculateNewStep();
    }

    void OldWalkEngine::reset() {
        uTorso.translation()     = Eigen::Vector2d({-footOffset[0], 0});
        uTorso.linear().matrix() = Eigen::Matrix2d::Identity();
        uLeftFoot.translation() = Eigen::Vector2d({0, kinematicsModel.leg.HIP_OFFSET_Y});
        uLeftFoot.linear().matrix() = Eigen::Matrix2d::Identity();
        uRightFoot.translation() = Eigen::Vector2d({0, -kinematicsModel.leg.HIP_OFFSET_Y});
        uRightFoot.linear().matrix() = Eigen::Matrix2d::Identity();

        uTorsoSource.matrix()          = Eigen::Matrix3d::Identity();
        uTorsoDestination.matrix()     = Eigen::Matrix3d::Identity();
        uLeftFootSource.matrix()       = Eigen::Matrix3d::Identity();
        uLeftFootDestination.matrix()  = Eigen::Matrix3d::Identity();
        uRightFootSource.matrix()      = Eigen::Matrix3d::Identity();
        uRightFootDestination.matrix() = Eigen::Matrix3d::Identity();

        velocityCurrent.matrix()    = Eigen::Matrix3d::Identity();
        velocityCommand.matrix()    = Eigen::Matrix3d::Identity();
        velocityDifference.matrix() = Eigen::Matrix3d::Identity();

        // gZMP exponential coefficients:
        zmpCoefficients = Eigen::Vector4d::Zero();
        zmpParams       = Eigen::Vector4d::Zero();

        // gGyro stabilization variables
        swingLeg      = swingLegInitial;
        beginStepTime = getTime();
        initialStep   = 2;

        // gStandard offset
        uLRFootOffset.translation() = Eigen::Vector2d({0, kinematicsModel.leg.HIP_OFFSET_Y - footOffset[1]});
        uLRFootOffset.linear().matrix() = Eigen::Matrix2d::Identity();

        // gWalking/Stepping transition variables
        startFromStep = false;

        state = State::STOPPED;

        // interrupted = false;
    }

    void OldWalkEngine::start() {
        if (state != State::WALKING) {
            swingLeg      = swingLegInitial;
            beginStepTime = getTime();
            initialStep   = 2;
            state         = State::WALKING;
        }
    }

    void OldWalkEngine::requestStop() {
        // always stops with feet together (which helps transition)
        if (state == State::WALKING) {
            state = State::STOP_REQUEST;
        }
    }

    void OldWalkEngine::stop() {
        state = State::STOPPED;
        // emit(std::make_unique<ActionPriorites>(ActionPriorites { subsumptionId, {
        // 0, 0 }})); // TODO: config
        log<NUClear::TRACE>("Walk Engine:: Stop request complete");
        emit(std::make_unique<WalkStopped>());
    }
    void OldWalkEngine::update(const Sensors& sensors) {
        double now = getTime();

        if (state == State::STOPPED) {
            updateStill(sensors);
            return;
        }

        // The phase of the current step, range: [0,1]
        double phase = (now - beginStepTime) / stepTime;

        bool newStep = false;

        if (phase > 1) {
            phase = std::fmod(phase, 1);
            beginStepTime += stepTime;
            newStep = true;
        }

        if (newStep && state == State::LAST_STEP) {
            stop();
            return;
        }

        // Compute FootSource and FootDestination for this step
        if (newStep) {
            calculateNewStep();
        }

        updateStep(phase, sensors);
    }

    void OldWalkEngine::updateStep(double phase, const Sensors& sensors) {
        // Get unitless phases for x and z motion
        Eigen::Vector3d foot = footPhase(phase, phase1Single, phase2Single);

        // Lift foot by amount depending on walk speed
        auto& limit = (velocityCurrent.translation()[0] > velocityHigh ? accelerationLimitsHigh
                                                          : accelerationLimits);  // TODO: use a function instead
        float speed = std::min(
            1.0,
            std::max(std::abs(velocityCurrent.translation()[0] / (limit[0] * 10)), std::abs(velocityCurrent.translation()[1] / (limit[1] * 10))));
        float scale = (step_height_fast_fraction - step_height_slow_fraction) * speed + step_height_slow_fraction;
        foot[2] *= scale;

        // don't lift foot at initial step, TODO: review
        if (initialStep > 0) {
            foot[2] = 0;
        }

        // Interpolate the transform from start to destination
        if (swingLeg == LimbID::RIGHT_LEG) {
            uRightFoot = interpolate(uRightFootSource, uRightFootDestination, foot[0]);
        }
        else {
            uLeftFoot = interpolate(uLeftFootSource, uLeftFootDestination, foot[0]);
        }
        // I hear you like arguments...
        uTorso = zmpCom(phase,
                        zmpCoefficients,
                        zmpParams,
                        stepTime,
                        zmpTime,
                        phase1Single,
                        phase2Single,
                        uSupport,
                        uLeftFootDestination,
                        uLeftFootSource,
                        uRightFootDestination,
                        uRightFootSource);

        Eigen::Affine3d leftFoot;
        leftFoot.translation() = Eigen::Vector3d({uLeftFoot.translation()[0], uLeftFoot.translation()[1], 0});
        leftFoot.linear() = Eigen::AngleAxisd(angle(uLeftFoot), Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Affine3d rightFoot;
        rightFoot.translation() = Eigen::Vector3d({uRightFoot.translation()[0], uRightFoot.translation()[1], 0});
        rightFoot.linear() = Eigen::AngleAxisd(angle(uRightFoot), Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // Lift swing leg
        if (swingLeg == LimbID::RIGHT_LEG) {
            rightFoot.linear().matrix() = Eigen::Matrix3d::Identity();
            rightFoot.translation() = Eigen::Vector3d({0, 0, stepHeight * foot[2]});
        }
        else {
            leftFoot.linear().matrix() = Eigen::Matrix3d::Identity();
            leftFoot.translation() = Eigen::Vector3d({0, 0, stepHeight * foot[2]});
        }

        Eigen::Affine2d hipOffsetTransform;
        hipOffsetTransform.translation() = Eigen::Vector2d({-kinematicsModel.leg.HIP_OFFSET_X, 0});
        hipOffsetTransform.linear().matrix() = Eigen::Matrix2d::Identity();
        Eigen::Affine2d uTorsoActual = localToWorld(uTorso, hipOffsetTransform);
        Eigen::Affine3d torso;
        torso.translation() = Eigen::Vector3d({uTorsoActual.translation()[0], uTorsoActual.translation()[1], bodyHeight});
        torso.linear().matrix() = Eigen::Matrix<double, 3, 3>::Identity();
        torso.rotate(Eigen::AngleAxis<double>(bodyTilt, Eigen::Vector3d::UnitY()));
        torso.rotate(Eigen::AngleAxis<double>(angle(uTorsoActual), Eigen::Vector3d::UnitX()));
        
        // Transform feet targets to be relative to the torso
        Eigen::Affine3d leftFootCOM  = worldToLocal(leftFoot, torso);
        Eigen::Affine3d rightFootCOM = worldToLocal(rightFoot, torso);

        // TODO: what is this magic?
        double phaseComp = std::min({1.0, foot[1] / 0.1, (1 - foot[1]) / 0.1});

        // Rotate foot around hip by the given hip roll compensation
        if (swingLeg == LimbID::LEFT_LEG) {
            rightFootCOM = 
            rightFootCOM = rotateZLocal(rightFootCOM, -hipRollCompensation * phaseComp,
                                                     Eigen::Affine3d(sensors.forward_kinematics[ServoID::R_HIP_ROLL]));
        }
        else {
            leftFootCOM = rotateZLocal(rightFootCOM, hipRollCompensation * phaseComp,
                                                   Eigen::Affine3d(sensors.forward_kinematics[ServoID::L_HIP_ROLL]));
        }

        if (balanceEnabled) {
            // Apply balance to our support foot
            balancer.balance(kinematicsModel,
                             swingLeg == LimbID::LEFT_LEG ? rightFootCOM.cast<float>() : leftFootCOM.cast<float>(),
                             swingLeg == LimbID::LEFT_LEG ? LimbID::RIGHT_LEG : LimbID::LEFT_LEG,
                             sensors);
        }

        // Assume the previous calculations were done in CoM space, now convert them to torso space
        // Height of CoM is assumed to be constant
        Eigen::Affine3f Htc;
        Htc.linear().matrix() = Eigen::Matrix3f::Identity();
        Htc.translation() = Eigen::Vector3f({-sensors.centre_of_mass[0], -sensors.centre_of_mass[1], 0.0f});

        Eigen::Affine3f leftFootTorso  = Htc * leftFootCOM.cast<float>();
        Eigen::Affine3f rightFootTorso = Htc * rightFootCOM.cast<float>();

        // Calculate roll and pitch compensation based on limits and compensation factors
        float rollComp  = clamp(-ankleRollLimit, ankleRollComp * sensors.angular_position[0], ankleRollLimit);
        float pitchComp = clamp(-anklePitchLimit, anklePitchComp * sensors.angular_position[1], anklePitchLimit);
        // Apply compensation to left and right feet positions
        leftFootTorso = rotateZ(legYaw, rotateY(pitchComp, rotateX(rollComp, leftFootTorso)));
        rightFootTorso = rotateZ(-legYaw, rotateY(pitchComp, rotateX(-rollComp, rightFootTorso)));

        std::vector<std::pair<ServoID, float>> joints;
        joints = calculateLegJoints(kinematicsModel, leftFootTorso, rightFootTorso);

        auto waypoints = motionLegs(joints);
        auto arms      = motionArms(phase);

        waypoints->insert(waypoints->end(), arms->begin(), arms->end());
        emit(std::move(waypoints));
    }

    std::unique_ptr<std::vector<ServoCommand>> OldWalkEngine::updateStillWayPoints(const Sensors& sensors) {
        uTorso = stepTorso(uLeftFoot, uRightFoot, 0.5);

        Eigen::Affine2d uTorsoActual;
        uTorsoActual.translation() = Eigen::Vector2d({-kinematicsModel.leg.HIP_OFFSET_X, 0});
        uTorsoActual.linear().matrix() = Eigen::Matrix2d::Identity();
        Eigen::Affine3d torso;
        torso.translation() = Eigen::Vector3d({uTorsoActual.translation()[0], uTorsoActual.translation()[1], bodyHeight});
        torso.linear().matrix() = Eigen::Matrix3d::Identity();
        torso.rotate(Eigen::AngleAxis<double>(bodyTilt, Eigen::Vector3d::UnitY()));
        torso.rotate(Eigen::AngleAxis<double>(angle(uTorsoActual), Eigen::Vector3d::UnitX()));
        
        // Transform feet targets to be relative to the torso
        Eigen::Affine3d leftFootCOM = worldToLocal(twoD_to_threeD(uLeftFoot), torso);
        Eigen::Affine3d rightFootCOM = worldToLocal(twoD_to_threeD(uRightFoot), torso);

        if (balanceEnabled) {
            // Apply balance to both legs when standing still
            balancer.balance(kinematicsModel, leftFootCOM.cast<float>(), LimbID::LEFT_LEG, sensors);
            balancer.balance(kinematicsModel, rightFootCOM.cast<float>(), LimbID::RIGHT_LEG, sensors);
        }

        // Assume the previous calculations were done in CoM space, now convert them to torso space
        // Height of CoM is assumed to be constant
        Eigen::Affine3f Htc;
        Htc.linear().matrix() = Eigen::Matrix3f::Identity();
        Htc.translation() = Eigen::Vector3f({-sensors.centre_of_mass[0], -sensors.centre_of_mass[1], 0.0f});

        Eigen::Affine3f leftFootTorso  = Htc * leftFootCOM.cast<float>();
        Eigen::Affine3f rightFootTorso = Htc * rightFootCOM.cast<float>();

        std::vector<std::pair<ServoID, float>> joints;
        joints = calculateLegJoints(kinematicsModel, leftFootTorso, rightFootTorso);

        auto waypoints = motionLegs(joints);

        auto arms = motionArms(0.5);
        waypoints->insert(waypoints->end(), arms->begin(), arms->end());

        return waypoints;
    }

    void OldWalkEngine::updateStill(const Sensors& sensors) {
        emit(std::move(updateStillWayPoints(sensors)));
    }

    std::unique_ptr<std::vector<ServoCommand>> OldWalkEngine::motionLegs(
        std::vector<std::pair<ServoID, float>> joints) {
        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(16);

        NUClear::clock::time_point time =
            NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);

        for (auto& joint : joints) {
            waypoints->push_back({subsumptionId,
                                  time,
                                  joint.first,
                                  joint.second,
                                  jointGains[joint.first],
                                  100});  // TODO: support separate gains for each leg
        }

        return waypoints;
    }

    std::unique_ptr<std::vector<ServoCommand>> OldWalkEngine::motionArms(double phase) {
        // Converts the phase into a sine wave that oscillates between 0 and 1 with a
        // period of 2 phases
        double easing = std::sin(M_PI * phase - M_PI / 2.0) / 2.0 + 0.5;
        if (swingLeg == LimbID::LEFT_LEG) {
            easing = -easing + 1.0;  // Gets the 2nd half of the sine wave
        }

        // Linearly interpolate between the start and end positions using the easing
        // parameter
        Eigen::Vector3d qLArmActual = easing * qLArmStart + (1.0 - easing) * qLArmEnd;
        Eigen::Vector3d qRArmActual = (1.0 - easing) * qRArmStart + easing * qRArmEnd;

        // Start arm/leg collision/prevention
        double rotLeftA           = normalizeAngle(angle(uLeftFoot) - angle(uTorso));
        double rotRightA          = normalizeAngle(angle(uTorso) - angle(uRightFoot));
        Eigen::Affine2d leftLegTorso  = worldToLocal(uTorso, uLeftFoot);
        Eigen::Affine2d rightLegTorso = worldToLocal(uTorso, uRightFoot);
        double leftMinValue       = 5 * M_PI / 180 + std::max(0.0, rotLeftA) / 2
                              + std::max(0.0, leftLegTorso.translation()[1] - 0.04) / 0.02 * (6 * M_PI / 180);
        double rightMinValue = -5 * M_PI / 180 - std::max(0.0, rotRightA) / 2
                               - std::max(0.0, -rightLegTorso.translation()[1] - 0.04) / 0.02 * (6 * M_PI / 180);
        // update shoulder pitch to move arm away from body
        qLArmActual[1] = std::max(leftMinValue, qLArmActual[1]);
        qRArmActual[1] = std::min(rightMinValue, qRArmActual[1]);
        // End arm/leg collision/prevention

        auto waypoints = std::make_unique<std::vector<ServoCommand>>();
        waypoints->reserve(6);

        NUClear::clock::time_point time =
            NUClear::clock::now() + std::chrono::nanoseconds(std::nano::den / UPDATE_FREQUENCY);
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::R_SHOULDER_PITCH,
                              float(qRArmActual[0]),
                              jointGains[ServoID::R_SHOULDER_PITCH],
                              100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::R_SHOULDER_ROLL,
                              float(qRArmActual[1]),
                              jointGains[ServoID::R_SHOULDER_ROLL],
                              100});
        waypoints->push_back(
            {subsumptionId, time, ServoID::R_ELBOW, float(qRArmActual[2]), jointGains[ServoID::R_ELBOW], 100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::L_SHOULDER_PITCH,
                              float(qLArmActual[0]),
                              jointGains[ServoID::L_SHOULDER_PITCH],
                              100});
        waypoints->push_back({subsumptionId,
                              time,
                              ServoID::L_SHOULDER_ROLL,
                              float(qLArmActual[1]),
                              jointGains[ServoID::L_SHOULDER_ROLL],
                              100});
        waypoints->push_back(
            {subsumptionId, time, ServoID::L_ELBOW, float(qLArmActual[2]), jointGains[ServoID::L_ELBOW], 100});

        return waypoints;
    }

    Eigen::Affine2d OldWalkEngine::stepTorso(Eigen::Affine2d uLeftFoot, Eigen::Affine2d uRightFoot, double shiftFactor) {
        Eigen::Affine2d footOffsetNeg;
        footOffsetNeg.translation() = -footOffset;
        footOffsetNeg.linear().matrix() = Eigen::Matrix2d::Identity();
        Eigen::Affine2d uLeftFootSupport  = localToWorld(uLeftFoot, footOffsetNeg);
        Eigen::Affine2d uRightFootSupport = localToWorld(uRightFoot, footOffsetNeg);
        return interpolate(uLeftFootSupport, uRightFootSupport, shiftFactor);
    }

    void OldWalkEngine::setVelocity(Eigen::Affine2d velocity) {
        // filter the commanded speed
        velocity.translation()[0]     = std::min(std::max(velocity.translation()[0], velocityLimits(0, 0)), velocityLimits(0, 1));
        velocity.translation()[1]     = std::min(std::max(velocity.translation()[1], velocityLimits(1, 0)), velocityLimits(1, 1));
        velocity.linear().matrix() = Eigen::Rotation2Dd(std::min(std::max(angle(velocity), velocityLimits(2, 0)), velocityLimits(2, 1))).toRotationMatrix();

        // slow down when turning
        double vFactor = 1 - std::abs(angle(velocity)) / accelerationTurningFactor;

        double stepMag   = std::sqrt(velocity.translation()[0] * velocity.translation()[0] + velocity.translation()[1] * velocity.translation()[1]);
        double magFactor = std::min(velocityLimits(0, 1) * vFactor, stepMag) / (stepMag + 0.000001);

        velocityCommand.translation()[0]     = velocity.translation()[0] * magFactor;
        velocityCommand.translation()[1]     = velocity.translation()[1] * magFactor;
        velocityCommand.linear() = velocity.linear();

        velocityCommand.translation()[0] = std::min(std::max(velocityCommand.translation()[0], velocityLimits(0, 0)), velocityLimits(0, 1));
        velocityCommand.translation()[1] = std::min(std::max(velocityCommand.translation()[1], velocityLimits(1, 0)), velocityLimits(1, 1));
        velocityCommand.linear() =
            Eigen::Rotation2Dd(std::min(std::max(angle(velocityCommand), velocityLimits(2, 0)), velocityLimits(2, 1))).toRotationMatrix();
    }

    Eigen::Affine2d OldWalkEngine::getVelocity() {
        return velocityCurrent;
    }

    Eigen::Vector2d OldWalkEngine::zmpSolve(double zs,
                                       double z1,
                                       double z2,
                                       double x1,
                                       double x2,
                                       double phase1Single,
                                       double phase2Single,
                                       double stepTime,
                                       double zmpTime) {
        /*
        Solves ZMP equations.
        The resulting form of x is
        x(t) = z(t) + aP*exp(t/zmpTime) + aN*exp(-t/zmpTime) -
        zmpTime*mi*sinh((t-Ti)/zmpTime) where the ZMP point is piecewise linear: z(0)
        = z1, z(T1 < t < T2) = zs, z(stepTime) = z2
        */
        double T1 = stepTime * phase1Single;
        double T2 = stepTime * phase2Single;
        double m1 = (zs - z1) / T1;
        double m2 = -(zs - z2) / (stepTime - T2);

        double c1       = x1 - z1 + zmpTime * m1 * std::sinh(-T1 / zmpTime);
        double c2       = x2 - z2 + zmpTime * m2 * std::sinh((stepTime - T2) / zmpTime);
        double expTStep = std::exp(stepTime / zmpTime);
        double aP       = (c2 - c1 / expTStep) / (expTStep - 1 / expTStep);
        double aN       = (c1 * expTStep - c2) / (expTStep - 1 / expTStep);
        return {aP, aN};
    }

    Eigen::Affine2d OldWalkEngine::zmpCom(double phase,
                                      Eigen::Vector4d zmpCoefficients,
                                      Eigen::Vector4d zmpParams,
                                      double stepTime,
                                      double zmpTime,
                                      double phase1Single,
                                      double phase2Single,
                                      Eigen::Affine2d uSupport,
                                      Eigen::Affine2d uLeftFootDestination,
                                      Eigen::Affine2d uLeftFootSource,
                                      Eigen::Affine2d uRightFootDestination,
                                      Eigen::Affine2d uRightFootSource) {
        Eigen::Affine2d com;
        com.matrix() = Eigen::Matrix3d::Identity();
        double expT     = std::exp(stepTime * phase / zmpTime);
        com.translation()[0]         = uSupport.translation()[0] + zmpCoefficients[0] * expT + zmpCoefficients[1] / expT;
        com.translation()[1]         = uSupport.translation()[1] + zmpCoefficients[2] * expT + zmpCoefficients[3] / expT;
        if (phase < phase1Single) {
            com.translation()[0] += zmpParams[0] * stepTime * (phase - phase1Single)
                       - zmpTime * zmpParams[0] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
            com.translation()[1] += zmpParams[1] * stepTime * (phase - phase1Single)
                       - zmpTime * zmpParams[1] * std::sinh(stepTime * (phase - phase1Single) / zmpTime);
        }
        else if (phase > phase2Single) {
            com.translation()[0] += zmpParams[2] * stepTime * (phase - phase2Single)
                       - zmpTime * zmpParams[2] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
            com.translation()[1] += zmpParams[3] * stepTime * (phase - phase2Single)
                       - zmpTime * zmpParams[3] * std::sinh(stepTime * (phase - phase2Single) / zmpTime);
        }
        // com[2] = .5 * (uLeftFoot[2] + uRightFoot[2]);
        // Linear speed turning
        com.linear() = Eigen::Rotation2Dd(phase * (angle(uLeftFootDestination) + angle(uRightFootDestination)) / 2
                      + (1 - phase) * (angle(uLeftFootSource) + angle(uRightFootSource)) / 2).toRotationMatrix();
        return com;
    }

    double OldWalkEngine::getTime() {
        return std::chrono::duration_cast<std::chrono::microseconds>(NUClear::clock::now().time_since_epoch()).count()
               * 1E-6;
    }

    double OldWalkEngine::procFunc(double value, double deadband, double maxvalue) {
        return std::abs(std::min(std::max(0.0, std::abs(value) - deadband), maxvalue));
    }

}  // namespace motion
}  // namespace module
