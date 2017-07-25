// Base class for all gait engines
// File: gait_engine.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include "GaitEngine.h"

#include <functional>

#include "utility/input/ServoID.h"
#include "utility/math/angle.h"
#include "utility/math/comparison.h"
#include "utility/math/filter/LinSinFillet.h"
#include "utility/math/filter/SlopeLimiter.h"
#include "utility/math/filter/SmoothDeadband.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/yaml_expression.h"

//
// GaitEngine class
//
namespace gait {

using extension::Configuration;

using ServoID = utility::input::ServoID;
using utility::nubugger::graph;
using utility::support::Expression;

GaitEngine::GaitEngine(NUClear::Reactor& reactor)
    : haltJointCmd()
    , haltJointEffort()
    , haltUseRawJointCmds(false)
    , reactor(reactor)
    , m_posX(0.0)
    , m_posY(0.0)
    , m_rotZ(0.0)
    , config()
    , m_gcvAccSmoothX(9)
    , m_gcvAccSmoothY(9)
    , m_gcvAccSmoothZ(9)
    , m_saveIFeedToHaltPose(false)
    , rxRobotModel()
    , rxModel()
    , mxModel()
    , txModel()
    , m_gcvZeroTime(0.0) {

    in.reset();
    out.reset();
    GaitEngine::updateHaltPose();
    GaitEngine::updateOdometry();

    // Reset the gait engine
    GaitEngine::reset();

    // Initialise the halt pose
    updateHaltPose();
    m_lastJointPose = m_jointHaltPose;

    // Set up callbacks for the local config parameters
    resetIntegrators();

    // Reset save integral feedback config parameter(s)
    resetSaveIntegrals();
}

void GaitEngine::updateConfig(const Configuration& config) {
    this->config = config;
    resizeFusedFilters(this->config["basicFusedFilterN"].as<float>());
    resizeDFusedFilters(this->config["basicDFusedFilterN"].as<float>());
    resizeIFusedFilters(this->config["basicIFusedFilterN"].as<float>());
    resizeGyroFilters(this->config["basicGyroFilterN"].as<float>());

    rxRobotModel.updateConfig(this->config);
    rxModel.updateConfig(this->config);
    mxModel.updateConfig(this->config);
    txModel.updateConfig(this->config);
}

// Reset function
void GaitEngine::reset() {
    // Reset the pose variables
    m_jointPose.reset();
    m_jointHaltPose.reset();
    m_lastJointPose.reset();
    m_inversePose.reset();
    m_abstractPose.reset();
    m_abstractHaltPose.reset();

    // Reset the gait command vector variables
    m_gcv.setZero();
    m_gcvInput.setZero();
    m_gcvDeriv.reset();
    m_gcvAccSmoothX.reset();
    m_gcvAccSmoothY.reset();
    m_gcvAccSmoothZ.reset();
    m_gcvAcc.setZero();

    // Reset the gait flags
    m_walk         = false;
    m_walking      = false;
    m_leftLegFirst = true;

    // Reset the step motion variables
    m_gaitPhase = 0.0;

    // Reset the blending
    resetBlending(USE_HALT_POSE);

    // Reset the capture step variables
    resetCaptureSteps(true);

    // Reset the integrators
    resetIntegrators();
}

void GaitEngine::resetBase() {
    in.reset();
    out.reset();
    updateHaltPose();
    m_posX = 0.0;
    m_posY = 0.0;
    m_rotZ = 0.0;
}

// Step function
void GaitEngine::step() {

    // Reset integrator flags
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;
    m_savedArmIFeed = false;
    m_savedLegIFeed = false;

    // Process the gait engine inputs
    processInputs();

    // Update the halt pose (updates m_abstractHaltPose, m_jointHaltPose and the inherited halt pose variables)
    updateHaltPose();

    // Retrieve the current blend factor
    double factor = blendFactor();

    // Generate the arm and leg motions if the gait is currently supposed to be walking
    if (m_walking)  // Robot is walking...
    {
        // Initialise the abstract pose to the halt pose
        m_abstractPose = m_abstractHaltPose;

        // Leg motions
        if (config["tuningNoLegs"].as<bool>()) {
            // Coerce the abstract leg poses
            clampAbstractLegPose(m_abstractPose.leftLeg);
            clampAbstractLegPose(m_abstractPose.rightLeg);

            // Convert legs: Abstract --> Joint
            m_jointPose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);
        }
        else {
            // Generate the abstract leg motion
            abstractLegMotion(m_abstractPose.leftLeg);
            abstractLegMotion(m_abstractPose.rightLeg);

            // Coerce the abstract leg poses
            clampAbstractLegPose(m_abstractPose.leftLeg);
            clampAbstractLegPose(m_abstractPose.rightLeg);

            // Convert legs: Abstract --> Inverse
            m_inversePose.setLegsFromAbstractPose(m_abstractPose.leftLeg, m_abstractPose.rightLeg);

            // Generate the inverse leg motion
            inverseLegMotion(m_inversePose.leftLeg);
            inverseLegMotion(m_inversePose.rightLeg);

            // Convert legs: Inverse --> Joint
            m_jointPose.setLegsFromInversePose(m_inversePose.leftLeg, m_inversePose.rightLeg);
        }

        // Arm motions
        if (config["tuningNoArms"].as<bool>()) {
            // Coerce the abstract arm poses
            clampAbstractArmPose(m_abstractPose.leftArm);
            clampAbstractArmPose(m_abstractPose.rightArm);

            // Convert arms: Abstract --> Joint
            m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
        }
        else {
            // Calculate the abstract arm motion
            abstractArmMotion(m_abstractPose.leftArm);
            abstractArmMotion(m_abstractPose.rightArm);

            // Coerce the abstract arm poses
            clampAbstractArmPose(m_abstractPose.leftArm);
            clampAbstractArmPose(m_abstractPose.rightArm);

            // Convert arms: Abstract --> Joint
            m_jointPose.setArmsFromAbstractPose(m_abstractPose.leftArm, m_abstractPose.rightArm);
        }

        // Pose blending
        if (factor != 0.0) m_jointPose.blendTowards(m_jointHaltPose, factor);

        // Save the last joint pose that was commanded during walking
        m_lastJointPose = m_jointPose;
    }
    else if (m_blending)  // Robot is blending but not walking...
    {
        // Reassert the last joint pose that was commanded during walking
        m_jointPose = m_lastJointPose;

        // Pose blending
        if (factor != 0.0) m_jointPose.blendTowards(m_jointHaltPose, factor);
    }
    else  // Robot is not walking or blending...
    {
        // Set the working joint pose to the halt pose
        m_jointPose = m_jointHaltPose;
    }

    // Update the gait engine outputs
    updateOutputs();

    // Reset the integrators if their offsets were saved to the halt pose
    if (m_saveIFeedToHaltPose) {
        m_saveIFeedToHaltPose = false;
        resetIntegrators();
    }

    // Plotting
    if (plot) {
        reactor.emit(graph("bonn/used_ifusedx", usedIFusedX));
        reactor.emit(graph("bonn/used_ifusedy", usedIFusedY));
        reactor.emit(graph("bonn/halt_blend_factor", factor));
    }
}

// Update the robot's halt pose
void GaitEngine::updateHaltPose() {
    // Calculate the required legAngleX with regard for motion stances
    double legAngleXFact = (config["enableMotionStances"].as<Expression>() ? m_motionLegAngleXFact : 1.0);
    double legAngleX     = legAngleXFact * config["haltLegAngleX"].as<Expression>()
                       + (1.0 - legAngleXFact) * config["haltLegAngleXNarrow"].as<Expression>();

    // Set whether to use raw joint commands
    haltUseRawJointCmds = !config["useServoModel"].as<bool>();

    // Set the halt pose for the legs
    m_abstractHaltPose.leftLeg.setPoseMirrored(config["haltLegExtension"].as<Expression>(),
                                               legAngleX,
                                               config["haltLegAngleY"].as<Expression>(),
                                               config["haltLegAngleZ"].as<Expression>());
    m_abstractHaltPose.rightLeg.setPoseMirrored(config["haltLegExtension"].as<Expression>(),
                                                legAngleX,
                                                config["haltLegAngleY"].as<Expression>(),
                                                config["haltLegAngleZ"].as<Expression>());
    m_abstractHaltPose.leftLeg.setFootPoseMirrored(config["haltFootAngleX"].as<Expression>(),
                                                   config["haltFootAngleY"].as<Expression>());
    m_abstractHaltPose.rightLeg.setFootPoseMirrored(config["haltFootAngleX"].as<Expression>(),
                                                    config["haltFootAngleY"].as<Expression>());

    // Set the halt pose for the arms
    m_abstractHaltPose.leftArm.setPoseMirrored(config["haltArmExtension"].as<Expression>(),
                                               config["haltArmAngleX"].as<Expression>(),
                                               config["haltArmAngleY"].as<Expression>());
    m_abstractHaltPose.rightArm.setPoseMirrored(config["haltArmExtension"].as<Expression>(),
                                                config["haltArmAngleX"].as<Expression>(),
                                                config["haltArmAngleY"].as<Expression>());

    // Apply the non-mirrored halt pose biases
    m_abstractHaltPose.leftLeg.angleX += config["haltLegAngleXBias"].as<Expression>();
    m_abstractHaltPose.rightLeg.angleX += config["haltLegAngleXBias"].as<Expression>();
    m_abstractHaltPose.leftLeg.footAngleX += config["haltFootAngleXBias"].as<Expression>();
    m_abstractHaltPose.rightLeg.footAngleX += config["haltFootAngleXBias"].as<Expression>();
    if (config["haltLegExtensionBias"].as<Expression>() >= 0.0) {
        m_abstractHaltPose.leftLeg.extension += config["haltLegExtensionBias"].as<Expression>();
    }
    else {
        m_abstractHaltPose.rightLeg.extension -= config["haltLegExtensionBias"].as<Expression>();
    }
    m_abstractHaltPose.leftArm.angleX += config["haltArmAngleXBias"].as<Expression>();
    m_abstractHaltPose.rightArm.angleX += config["haltArmAngleXBias"].as<Expression>();

    // Set the halt joint efforts for the arms
    m_abstractHaltPose.leftArm.cad.setEffort(config["haltEffortArm"].as<Expression>());
    m_abstractHaltPose.rightArm.cad.setEffort(config["haltEffortArm"].as<Expression>());

    // Set the halt joint efforts for the legs
    m_abstractHaltPose.leftLeg.cld.setEffort(config["haltEffortHipYaw"].as<Expression>(),
                                             config["haltEffortHipRoll"].as<Expression>(),
                                             config["haltEffortHipPitch"].as<Expression>(),
                                             config["haltEffortKneePitch"].as<Expression>(),
                                             config["haltEffortAnklePitch"].as<Expression>(),
                                             config["haltEffortAnkleRoll"].as<Expression>());
    m_abstractHaltPose.rightLeg.cld.setEffort(config["haltEffortHipYaw"].as<Expression>(),
                                              config["haltEffortHipRoll"].as<Expression>(),
                                              config["haltEffortHipPitch"].as<Expression>(),
                                              config["haltEffortKneePitch"].as<Expression>(),
                                              config["haltEffortAnklePitch"].as<Expression>(),
                                              config["haltEffortAnkleRoll"].as<Expression>());

    // Set the halt pose support coefficients
    m_abstractHaltPose.leftLeg.cld.setSupportCoeff(0.5);
    m_abstractHaltPose.rightLeg.cld.setSupportCoeff(0.5);

    // Set the link lengths
    m_abstractHaltPose.setLinkLengths(config["legLinkLength"].as<Expression>(),
                                      config["armLinkLength"].as<Expression>());

    // Convert the clamped halt pose into the joint representation (abstract poses derived from m_abstractHaltPose
    // are
    // clamped later, always right before they are converted into another space, i.e. joint/inverse)
    pose::AbstractPose clampedAbstractHaltPose = m_abstractHaltPose;
    clampAbstractPose(clampedAbstractHaltPose);
    m_jointHaltPose.setFromAbstractPose(clampedAbstractHaltPose);

    // Transcribe the joint halt pose to the required halt pose arrays
    haltJointCmd    = m_jointHaltPose.writeJointPosArray();
    haltJointEffort = m_jointHaltPose.writeJointEffortArray();
}

// Set the robot's odometry
void GaitEngine::setOdometry(double posX, double posY, double rotZ) {
    // Update the odometry of the internal robot model object
    // Note: rotZ is interpreted as a fused yaw angle relative to the global fixed frame
    rxRobotModel.setOdom(posX, posY, rotZ);
}

// Update the robot's odometry
void GaitEngine::updateOdometry() {
    // Transcribe the position odometry information
    Eigen::Vector3d trunkPos = rxRobotModel.trunkLink.position();
    out.odomPosition[0]      = trunkPos.x();
    out.odomPosition[1]      = trunkPos.y();
    out.odomPosition[2]      = trunkPos.z();

    // Transcribe the orientation odometry information
    Eigen::Quaterniond trunkRot = rxRobotModel.trunkLink.orientation();
    out.odomOrientation[0]      = trunkRot.w();
    out.odomOrientation[1]      = trunkRot.x();
    out.odomOrientation[2]      = trunkRot.y();
    out.odomOrientation[3]      = trunkRot.z();
}

// Reset function for the integrators
void GaitEngine::resetIntegrators() {
    // Reset the integrators
    iFusedXFeedIntegrator.reset();
    iFusedYFeedIntegrator.reset();
    iFusedXFeedFilter.reset();
    iFusedYFeedFilter.reset();
    iFusedXLastTime = 0.0;
    iFusedYLastTime = 0.0;
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;
}

// Reset function for the config parameter(s) that save the integrated feedback values
void GaitEngine::resetSaveIntegrals() {
    // Reset the required config parameter(s) to false
    m_saveIFeedToHaltPose = false;
}

// Reset function for capture steps functionality
void GaitEngine::resetCaptureSteps(bool resetRobotModel) {
    // Reset basic feedback filters
    fusedXFeedFilter.reset();
    fusedYFeedFilter.reset();
    dFusedXFeedFilter.reset();
    dFusedYFeedFilter.reset();
    iFusedXFeedFilter.reset();
    iFusedYFeedFilter.reset();
    gyroXFeedFilter.reset();
    gyroYFeedFilter.reset();

    // Reset the integrator variables (Note: We don't reset the integrators themselves here as they have their own
    // time-based reset mechanism)
    haveIFusedXFeed = false;
    haveIFusedYFeed = false;
    usedIFusedX     = false;
    usedIFusedY     = false;
    m_savedArmIFeed = false;
    m_savedLegIFeed = false;

    // Reset basic feedback variables
    fusedXFeed  = 0.0;
    fusedYFeed  = 0.0;
    dFusedXFeed = 0.0;
    dFusedYFeed = 0.0;
    iFusedXFeed = 0.0;
    iFusedYFeed = 0.0;
    gyroXFeed   = 0.0;
    gyroYFeed   = 0.0;

    // Reset capture step objects
    if (resetRobotModel) {
        rxRobotModel.reset(resetRobotModel);
        rxRobotModel.setSupportLeg(m_leftLegFirst ? 1 : -1);  // Left leg first in the air means that the first support
                                                              // leg is the right leg, 1 in the margait/contrib sign
                                                              // convention
        rxRobotModel.supportExchangeLock = true;
    }
    rxModel.reset();
    mxModel.reset();
    txModel.reset();
    limp.reset();
    m_comFilter.reset(in.nominaldT);

    // Reset capture step variables
    adaptation.x()         = 1.0;
    adaptation.y()         = 1.0;
    lastSupportOrientation = 0.0;
    oldGcvTargetY          = 0.0;
    virtualSlope           = 0.0;
    stepTimeCount          = 0.0;
    lastStepDuration       = 0.0;
    stepCounter            = 0;
    noCLStepsCounter       = std::max((int) (m_gcvZeroTime / in.nominaldT + 0.5), 1);
    resetCounter           = 100;  // Force Mx to match Rx completely for the next/first few cycles
    cycleNumber            = 0;

    // Reset the motion stance variables
    if (resetRobotModel) {
        resetMotionStance();
    }

    // Reset save integral feedback config parameter
    resetSaveIntegrals();
}

// Reset walking function (this should be the only function that modifies the m_walking flag)
void GaitEngine::resetWalking(bool walking, const Eigen::Vector3d& gcvBias) {
    // Reset variables
    m_walking      = walking;
    m_gcv          = gcvBias;
    m_gaitPhase    = 0.0;
    m_leftLegFirst = config["leftLegFirst"].as<bool>();

    // Initialise the blending
    if (m_walking) {
        resetBlending(USE_HALT_POSE);
        setBlendTarget(USE_CALC_POSE, config["startBlendPhaseLen"].as<Expression>());
    }
    else {
        setBlendTarget(USE_HALT_POSE, config["stopBlendPhaseLen"].as<Expression>());
    }

    // Reset the capture step variables
    resetCaptureSteps(walking);
}

// Process inputs function
void GaitEngine::processInputs() {
    // Transcribe whether it is desired of us to walk right now
    m_walk = in.gaitCmd.walk;

    // Transcribe and bias the desired gait velocity
    Eigen::Vector3d gcvBias(config["gcvBiasLinVelX"].as<Expression>(),
                            config["gcvBiasLinVelY"].as<Expression>(),
                            config["gcvBiasAngVelZ"].as<Expression>());
    if (m_walk) {
        m_gcvInput = gcvBias + Eigen::Vector3d(in.gaitCmd.linVelX, in.gaitCmd.linVelY, in.gaitCmd.angVelZ);
    }
    else {
        m_gcvInput = gcvBias;
    }

    // Check whether we should start walking
    if (m_walk && !m_walking) {
        resetWalking(true, gcvBias);
    }

    // Perform update tasks if walking is active
    if (m_walking) {
        updateRobot(gcvBias);
    }
    else if (m_blending) {
        m_blendPhase += config["gaitFrequency"].as<Expression>() * M_PI
                        * in.nominaldT;  // Increment the blend phase by the nominal gait
                                         // frequency (blend phase updates usually happen
                                         // inside updateRobot())
    }

    // Plot data
    if (plot) {
        reactor.emit(graph("bonn/gcv", m_gcv.x(), m_gcv.y(), m_gcv.z()));
        reactor.emit(graph("bonn/gcv_acc", m_gcvAcc.x(), m_gcvAcc.y(), m_gcvAcc.z()));
        reactor.emit(graph("bonn/gcv_phase", m_gaitPhase));
    }
}

// Update robot function
void GaitEngine::updateRobot(const Eigen::Vector3d& gcvBias) {
    // Note: Coming into this function we assume that we have in, m_walk and m_gcvInput available and up to date.
    //       Other class member variables such as m_gcv etc should naturally also be available and have their
    //       values from the last update (or other, if for example this is being called fresh from a reset).

    //
    // Preliminary work
    //

    // Increment the cycle number
    cycleNumber++;

    // Transcribe the gcv input
    Eigen::Vector3f gcvInput = m_gcvInput.cast<float>();

    // Calculate the current unbiased gait command vector
    Eigen::Vector3d gcvUnbiased = m_gcv - gcvBias;  // Should only be used for checking the proximity of gcv to a
    // velocity command of zero (as the input to the gait engine sees
    // it)

    // Retrieve the system iteration time and true time dT
    double systemIterationTime = in.nominaldT;
    double truedT              = in.truedT;
    stepTimeCount += truedT;

    // Retrieve the robot's fused angle
    const double fusedX = in.Htw(2, 1) + config["mgFusedOffsetX"].as<Expression>();
    const double fusedY = in.Htw(2, 0) + config["mgFusedOffsetY"].as<Expression>();

    // Update the required input data for the limp models
    rxModel.updateInputData(systemIterationTime, fusedX, fusedY);
    mxModel.updateInputData(systemIterationTime, fusedX, fusedY);
    txModel.updateInputData(systemIterationTime, fusedX, fusedY);

    // Make local aliases of frequently used config variables
    const double C     = config["mgC"].as<Expression>();
    const double alpha = config["mgAlpha"].as<Expression>();  // Lateral step apex size.
    const double delta = config["mgDelta"].as<Expression>();  // Minimum lateral support exchange location.
    const double omega = config["mgOmega"].as<Expression>();  // Maximum lateral support exchange location.
    const double sigma = config["mgSigma"].as<Expression>();  // Maximum sagittal apex velocity.

    //
    // State estimation
    //

    // What we need is the com trajectory in the support foot frame, which is naturally discontinuous.
    // When a support exchange occurs, the com state has to be transformed into the new support frame.
    // This includes a rotation of the com velocity vector by the angle of the new support foot with
    // respect to the old support foot.

    // Retrieve the robot's current measured pose in terms of joint angles and populate a Pose struct with it
    // Note: We only need to populate the x, y, z fields for each joint, as this is all that our rxRobotModel needs.
    contrib::Pose measuredPose;
    measuredPose.headPose.neck.setPos(0.0, 0.0, 0.0);
    measuredPose.trunkPose.spine.setPos(0.0, 0.0, 0.0);
    measuredPose.leftArmPose.shoulder.setPos(
        in.jointPos[ServoID::L_SHOULDER_ROLL], in.jointPos[ServoID::L_SHOULDER_PITCH], 0.0);
    measuredPose.leftArmPose.elbow.setPos(0.0, in.jointPos[ServoID::L_ELBOW], 0.0);
    measuredPose.leftLegPose.hip.setPos(
        in.jointPos[ServoID::L_HIP_ROLL], in.jointPos[ServoID::L_HIP_PITCH], in.jointPos[ServoID::L_HIP_YAW]);
    measuredPose.leftLegPose.knee.setPos(0.0, in.jointPos[ServoID::L_KNEE], 0.0);
    measuredPose.leftLegPose.ankle.setPos(in.jointPos[ServoID::L_ANKLE_ROLL], in.jointPos[ServoID::L_ANKLE_PITCH], 0.0);
    measuredPose.rightArmPose.shoulder.setPos(
        in.jointPos[ServoID::R_SHOULDER_ROLL], in.jointPos[ServoID::R_SHOULDER_PITCH], 0.0);
    measuredPose.rightArmPose.elbow.setPos(0.0, in.jointPos[ServoID::R_ELBOW], 0.0);
    measuredPose.rightLegPose.hip.setPos(
        in.jointPos[ServoID::R_HIP_ROLL], in.jointPos[ServoID::R_HIP_PITCH], in.jointPos[ServoID::R_HIP_YAW]);
    measuredPose.rightLegPose.knee.setPos(0.0, in.jointPos[ServoID::R_KNEE], 0.0);
    measuredPose.rightLegPose.ankle.setPos(
        in.jointPos[ServoID::R_ANKLE_ROLL], in.jointPos[ServoID::R_ANKLE_PITCH], 0.0);

    // Update the rx robot model and filtered CoM state
    rxRobotModel.update(measuredPose, fusedX, fusedY);
    Eigen::Vector3d suppComVector = rxRobotModel.suppComVector();
    if (cycleNumber == 1) {
        m_comFilter.reset(systemIterationTime, suppComVector.x(), suppComVector.y(), 0.0, 0.0);
    }
    m_comFilter.update(suppComVector.x(), suppComVector.y());

    // If this is the first cycle then reset the odometry properly as we are now in pose
    if (cycleNumber == 1) {
        rxRobotModel.setOdom(0.0, 0.0, 0.0);
    }

    // Update the rx model timing
    rxModel.timeSinceStep += systemIterationTime;
    rxModel.nominalTimeToStep -= systemIterationTime;

    // Handle rx robot model support exchange
    if (rxRobotModel.supportExchange) {
        double supportOrientation = rxRobotModel.fusedYaw(rxRobotModel.suppFootstep.orientation());
        double angleFromLastToNewSupport =
            utility::math::angle::normalizeAngle(supportOrientation - lastSupportOrientation);
        lastSupportOrientation = supportOrientation;

        // To maintain continuous velocity in the world frame, rotate the CoM velocity vector into the new support
        // frame.
        Eigen::Matrix2f rot;
        // clang-format off
        // 2D rotation by (-angleFromLastToNewSupport)
        rot <<  std::cos(angleFromLastToNewSupport), std::sin(angleFromLastToNewSupport)
               -std::sin(angleFromLastToNewSupport), std::cos(angleFromLastToNewSupport);
        // clang-format on
        Eigen::Vector2f v(rxModel.vx, rxModel.vy);
        v = rot * v;

        m_comFilter.reset(systemIterationTime, suppComVector.x(), suppComVector.y(), v.x(), v.y());
        rxModel.timeSinceStep     = 0;
        rxModel.nominalTimeToStep = 2.0 * rxModel.nominalFootStepTHalf;
        stepCounter++;
        lastStepDuration = stepTimeCount;
        stepTimeCount    = 0.0;
    }

    // Add CoM offsets
    contrib::LimpState ms(m_comFilter.x(), m_comFilter.y(), m_comFilter.vx(), m_comFilter.vy(), 0.0, 0.0);
    ms.supportLegSign = rxRobotModel.supportLegSign;
    ms.x += config["mgComOffsetX"].as<Expression>();
    ms.y += ms.supportLegSign * config["mgComOffsetY"].as<Expression>() + config["mgComOffsetYBias"].as<Expression>();

    // The LimpState ms concludes the estimation of the "CoM" state using direct sensor input and the kinematic
    // model.
    // Let's set the rx limp model.
    rxModel.setState(ms, gcvInput);  // TODO: Should this be gcvInput or m_gcv?

    //
    // Adaptation gate
    //

    // A blending factor between 0 and 1 is used to blend between the rx state and a simulated ideal LIP state.
    // When the loop close factor is 1, the loop is closed completely and the model state (mx) will be
    // equal to the rx state. When the loop close factor is 0, sensor input is ignored and the
    // mx state is entirely driven by an ideal pendulum model. The factor interpolates between the sensor
    // input and an idealized pendulum model. We use this to inhibit adaptation near the steps.

    // Forward and update the mxModel
    mxModel.forwardThroughStep(systemIterationTime);
    contrib::LimpState mx = mxModel.getMotionState();
    contrib::LimpState rx = rxModel.getMotionState();
    contrib::LimpState ls = mx;
    double adaptx         = utility::math::clamp(0.0f, adaptation.x(), 1.0f);
    double adapty         = utility::math::clamp(0.0f, adaptation.y(), 1.0f);
    if (rxModel.supportLegSign * mxModel.supportLegSign > 0) {
        ls.x  = (1.0 - adaptx) * mx.x + adaptx * rx.x;
        ls.vx = (1.0 - adaptx) * mx.vx + adaptx * rx.vx;
        ls.y  = (1.0 - adapty) * mx.y + adapty * rx.y;
        ls.vy = (1.0 - adapty) * mx.vy + adapty * rx.vy;
    }
    mxModel.setState(ls, gcvInput);  // TODO: Should this be gcvInput or m_gcv?

    // Latency preview (beyond here we are working config["mgLatency"].as<Expression>() seconds into the future with
    // all
    // our
    // calculations!)
    txModel        = mxModel;
    double latency = config["mgLatency"].as<Expression>();
    if (mxModel.timeToStep > latency) {
        txModel.forward(latency);
    }
    else {
        txModel.forward(mxModel.timeToStep);
        txModel.step();
        txModel.forward(latency - mxModel.timeToStep);
    }

    //
    // Noise suppression
    //

    // The step noise suppression factor is implemented as a Gaussian function. It's 0 near the support exchange
    // to inhibit problematic phases of state estimation. It needs to be computed after the step reset to avoid
    // jumps.
    // Expectation adaptation. The idea is that we stay open loop when everything is as expected,
    // and only adapt to sensor input when it's far away from an expected value. It inhibits the adaptation rate a
    // little bit, but it has a nice smoothing effect. Nominal adaptation. With the nominal adaptation we can
    // inhibit
    // adaptation when we are near the "optimal" nominal state.

    // Determine whether the loop should be closed
    bool loopClosed = config["cmdUseRXFeedback"].as<bool>() && (rxModel.supportLegSign == mxModel.supportLegSign);

    // Initialise the adaptation terms (Note: The initial value is an implicit scale of the entire adaptation term)
    double adaptationX = (loopClosed ? double(config["nsAdaptationGain"].as<Expression>()) : 0.0);
    double adaptationY = (loopClosed ? double(config["nsAdaptationGain"].as<Expression>()) : 0.0);

    // Calculate step noise suppression factor
    double phase = std::max(std::min(txModel.timeToStep, std::min(mxModel.timeToStep, mxModel.timeSinceStep)), 0.0);
    double stepNoiseSuppression = 1.0 - std::exp(-(phase * phase) / (2.0 * config["nsStepNoiseTime"].as<Expression>()
                                                                     * config["nsStepNoiseTime"].as<Expression>()));
    adaptationX *= stepNoiseSuppression;
    adaptationY *= stepNoiseSuppression;

    // If we are near the expected state, there is no need for adaptation.
    Eigen::Vector2f expectationDeviation;
    expectationDeviation.x() =
        config["nsGain"].as<Expression>()
        * (Eigen::Vector2f(rxModel.x, rxModel.vx) - Eigen::Vector2f(mxModel.x, mxModel.vx)).norm();
    expectationDeviation.y() =
        config["nsGain"].as<Expression>()
        * (Eigen::Vector2f(rxModel.y, rxModel.vy) - Eigen::Vector2f(mxModel.y, mxModel.vy)).norm();
    adaptationX *= expectationDeviation.x();
    adaptationY *= expectationDeviation.y();

    //  // If our fused angle is within a certain nominal range then don't adapt
    //  Eigen::Vector2f fusedAngleAdaptation;
    //  fusedAngleAdaptation.x = ((fusedY <= config["nsFusedYRangeLBnd"].as<Expression>() || fusedY >=
    //  config["nsFusedYRangeUBnd"].as<Expression>()) ? 1.0
    //  :
    //  0.0); fusedAngleAdaptation.y = ((fusedX <= config["nsFusedXRangeLBnd"].as<Expression>() || fusedX >=
    //  config["nsFusedXRangeUBnd"].as<Expression>())
    //  ? 1.0 : 0.0); adaptationX *= fusedAngleAdaptation.x; adaptationY *= fusedAngleAdaptation.y;

    //  // If we are near the nominal state, there is no need for adaptation.
    //  Eigen::Vector2f nominalAdaptation;
    //  nominalAdaptation.x = (qAbs(rxModel.energyX - rxModel.nominalState.energyX) >
    //  config["nsMinDeviation"].as<Expression>() ?
    //  config["nsGain"].as<Expression>() * qAbs(rxModel.energyX - rxModel.nominalState.energyX) : 0.0);
    //  nominalAdaptation.y =
    //  (qAbs(rxModel.energyY - rxModel.nominalState.energyY) > config["nsMinDeviation"].as<Expression>() ?
    //  config["nsGain"].as<Expression>() *
    //  qAbs(rxModel.energyY - rxModel.nominalState.energyY) : 0.0);

    //  // If the fused angle is near zero, there is no need for adaptation.
    //  Eigen::Vector2f fusedAngleAdaptation;
    //  fusedAngleAdaptation.x = (absFusedY > config["nsMinDeviation"].as<Expression>() ?
    //  config["nsGain"].as<Expression>() * absFusedY : 0.0);
    //  fusedAngleAdaptation.y = (absFusedX > config["nsMinDeviation"].as<Expression>() ?
    //  config["nsGain"].as<Expression>() * absFusedX : 0.0);

    // Calculate the required adaptation (0.0 => Completely trust Mx, 1.0 => Completely trust Rx)
    adaptation.x() = utility::math::clamp(0.0, adaptationX, double(config["nsMaxAdaptation"].as<Expression>()));
    adaptation.y() = utility::math::clamp(0.0, adaptationY, double(config["nsMaxAdaptation"].as<Expression>()));

    // Adaptation resetting (force complete trust in Rx)
    if (resetCounter > 0) {
        adaptation.x() = 1.0;
        adaptation.y() = 1.0;
        resetCounter--;
    }

    //
    // Basic feedback mechanisms
    //

    // Calculate the fused angle deviation from expected
    double expectedFusedX = config["basicFusedExpXSinOffset"].as<Expression>()
                            + config["basicFusedExpXSinMag"].as<Expression>()
                                  * std::sin(m_gaitPhase - config["basicFusedExpXSinPhase"].as<Expression>());
    double expectedFusedY = config["basicFusedExpYSinOffset"].as<Expression>()
                            + config["basicFusedExpYSinMag"].as<Expression>()
                                  * std::sin(m_gaitPhase - config["basicFusedExpYSinPhase"].as<Expression>());
    double deviationFusedX = fusedX - expectedFusedX;
    double deviationFusedY = fusedY - expectedFusedY;

    // Calculate the gyro deviation from expected
    double expectedGyroX  = config["basicGyroExpX"].as<Expression>();
    double expectedGyroY  = config["basicGyroExpY"].as<Expression>();
    double deviationGyroX = in.gyroscope.x() - expectedGyroX;
    double deviationGyroY = in.gyroscope.y() - expectedGyroY;

    // Calculate basic fused angle deviation feedback values
    fusedXFeedFilter.put(deviationFusedX);
    fusedYFeedFilter.put(deviationFusedY);
    fusedXFeed = config["basicFusedGainAllLat"].as<Expression>()
                 * utility::math::filter::SmoothDeadband::eval(fusedXFeedFilter.value(),
                                                               config["basicFusedDeadRadiusX"].as<Expression>());
    fusedYFeed = config["basicFusedGainAllSag"].as<Expression>()
                 * utility::math::filter::SmoothDeadband::eval(fusedYFeedFilter.value(),
                                                               config["basicFusedDeadRadiusY"].as<Expression>());
    if (!config["basicFusedEnabledLat"].as<bool>()) {
        fusedXFeed = 0.0;
    }
    if (!config["basicFusedEnabledSag"].as<bool>()) {
        fusedYFeed = 0.0;
    }

    // Calculate basic fused angle deviation derivative feedback values
    // TODO: Modify the weighting based on gait phase to reject known fused angle bumps
    dFusedXFeedFilter.addXYW(in.timestamp, deviationFusedX, 1.0);
    // TODO: Modify the weighting based on gait phase to reject known fused angle bumps
    dFusedYFeedFilter.addXYW(in.timestamp, deviationFusedY, 1.0);
    dFusedXFeed = config["basicDFusedGainAllLat"].as<Expression>()
                  * utility::math::filter::SmoothDeadband::eval(dFusedXFeedFilter.deriv(),
                                                                config["basicDFusedDeadRadiusX"].as<Expression>());
    dFusedYFeed = config["basicDFusedGainAllSag"].as<Expression>()
                  * utility::math::filter::SmoothDeadband::eval(dFusedYFeedFilter.deriv(),
                                                                config["basicDFusedDeadRadiusY"].as<Expression>());
    if (!config["basicDFusedEnabledLat"].as<bool>()) {
        dFusedXFeed = 0.0;
    }
    if (!config["basicDFusedEnabledSag"].as<bool>()) {
        dFusedYFeed = 0.0;
    }

    // Update the half life time of the fused angle deviation integrators
    double halfLifeCycles = config["basicIFusedHalfLifeTime"].as<Expression>() / systemIterationTime;
    iFusedXFeedIntegrator.setHalfLife(halfLifeCycles);
    iFusedYFeedIntegrator.setHalfLife(halfLifeCycles);

    // Integrate up the deviation in the fused angle X
    double timeSinceIFusedXFeedback = in.timestamp - iFusedXLastTime;
    if (timeSinceIFusedXFeedback < config["basicIFusedTimeToFreeze"].as<Expression>()) {  // Integrate normally
        iFusedXFeedIntegrator.integrate(deviationFusedX);
    }
    else if (timeSinceIFusedXFeedback
             > config["basicIFusedTimeToDecay"].as<Expression>()) {  // Decay the value of the integral to zero
                                                                     // (integral is exponentially weighted)
        iFusedXFeedIntegrator.integrate(0.0);
    }

    // Integrate up the deviation in the fused angle Y
    double timeSinceIFusedYFeedback = in.timestamp - iFusedYLastTime;
    if (timeSinceIFusedYFeedback < config["basicIFusedTimeToFreeze"].as<Expression>()) {  // Integrate normally
        iFusedYFeedIntegrator.integrate(deviationFusedY);
    }
    else if (timeSinceIFusedYFeedback
             > config["basicIFusedTimeToDecay"].as<Expression>()) {  // Decay the value of the integral to zero
                                                                     // (integral is exponentially weighted)
        iFusedYFeedIntegrator.integrate(0.0);
    }

    // Calculate a basic fused angle deviation integral feedback value
    iFusedXFeedFilter.put(iFusedXFeedIntegrator.integral());
    iFusedYFeedFilter.put(iFusedYFeedIntegrator.integral());
    iFusedXFeed = 1e-3 * config["basicIFusedGainAllLat"].as<Expression>() * iFusedXFeedFilter.value();
    iFusedYFeed = 1e-3 * config["basicIFusedGainAllSag"].as<Expression>() * iFusedYFeedFilter.value();
    if (!config["basicIFusedEnabledLat"].as<bool>()) {
        iFusedXFeed = 0.0;
    }
    if (!config["basicIFusedEnabledSag"].as<bool>()) {
        iFusedYFeed = 0.0;
    }
    haveIFusedXFeed =
        (config["basicIFusedGainAllLat"].as<Expression>() != 0.0 && config["basicIFusedEnabledLat"].as<bool>());
    haveIFusedYFeed =
        (config["basicIFusedGainAllSag"].as<Expression>() != 0.0 && config["basicIFusedEnabledSag"].as<bool>());

    // Calculate basic gyro deviation feedback values
    // TODO: Modify the weighting based on gait phase to reject known gyro bumps
    gyroXFeedFilter.addXYW(in.timestamp, deviationGyroX, 1.0);
    // TODO: Modify the weighting based on gait phase to reject known gyro bumps
    gyroYFeedFilter.addXYW(in.timestamp, deviationGyroY, 1.0);
    gyroXFeed = config["basicGyroGainAllLat"].as<Expression>()
                * utility::math::filter::SmoothDeadband::eval(gyroXFeedFilter.value(),
                                                              config["basicGyroDeadRadiusX"].as<Expression>());
    gyroYFeed = config["basicGyroGainAllSag"].as<Expression>()
                * utility::math::filter::SmoothDeadband::eval(gyroYFeedFilter.value(),
                                                              config["basicGyroDeadRadiusY"].as<Expression>());
    if (!config["basicGyroEnabledLat"].as<bool>()) {
        gyroXFeed = 0.0;
    }
    if (!config["basicGyroEnabledSag"].as<bool>()) {
        gyroYFeed = 0.0;
    }

    // Calculate a modification to the gait phase frequency based on basic feedback terms
    double timingFeedWeight =
        utility::math::clamp(-1.0,
                             -config["basicTimingWeightFactor"].as<Expression>()
                                 * sin(m_gaitPhase - 0.5 * config["doubleSupportPhaseLen"].as<Expression>()),
                             1.0);
    double timingFeed = utility::math::filter::SmoothDeadband::eval(fusedXFeedFilter.value() * timingFeedWeight,
                                                                    config["basicTimingFeedDeadRad"].as<Expression>());
    double timingFreqDelta = (timingFeed >= 0.0 ? config["basicTimingGainSpeedUp"].as<Expression>() * timingFeed
                                                : config["basicTimingGainSlowDown"].as<Expression>() * timingFeed);
    if (!(config["basicEnableTiming"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
        timingFreqDelta = 0.0;
    }

    // Virtual slope
    virtualSlope = 0.0;
    if (config["basicEnableVirtualSlope"].as<bool>() && config["basicGlobalEnable"].as<bool>()) {
        virtualSlope  = config["virtualSlopeOffset"].as<Expression>();
        double dev    = fusedY - config["virtualSlopeMidAngle"].as<Expression>();
        double absdev = std::abs(dev);
        if (absdev > config["virtualSlopeMinAngle"].as<Expression>()) {
            virtualSlope += utility::math::sign(dev)
                            * (dev * config["gcvPrescalerLinVelX"].as<Expression>() * m_gcv.x() > 0
                                   ? config["virtualSlopeGainAsc"].as<Expression>()
                                   : config["virtualSlopeGainDsc"].as<Expression>())
                            * (absdev - config["virtualSlopeMinAngle"].as<Expression>());
        }
    }

    //
    // Gait phase update
    //

    // Save the value of the gait phase before the update
    double oldGaitPhase = m_gaitPhase;

    // Get the support leg sign according to the current gait phase
    int gaitPhaseLegSign = utility::math::sign(m_gaitPhase);

    // By default use the nominal gait frequency for timing (these variables tell us on what leg we should be standing,
    // for how much gait phase to come, traversing the gait phase with what frequency)
    int supportLegSign        = gaitPhaseLegSign;
    double gaitFrequency      = config["gaitFrequency"].as<Expression>();
    double remainingGaitPhase = (m_gaitPhase > 0.0 ? M_PI - m_gaitPhase : 0.0 - m_gaitPhase);
    if (remainingGaitPhase < 1e-8) {
        remainingGaitPhase = 1e-8;
    }

    // Handle the case of closed loop timing feedback
    if (config["cmdUseCLTiming"].as<bool>()) {
        // If desired, use the TX model timing
        if (config["cmdUseTXTiming"].as<bool>()) {
            supportLegSign = txModel.supportLegSign;
            if (supportLegSign != gaitPhaseLegSign) {  // Premature step case - happens even to the best of us...
                remainingGaitPhase += M_PI;  // The txModel.timeToStep variable tells us how much time we should take
                                             // until the end of the step *after* the one we're currently taking, so
                                             // that's an extra PI radians to cover
            }
            if (txModel.timeToStep > 1e-8) {
                gaitFrequency = remainingGaitPhase / (M_PI * txModel.timeToStep);  // Note: Always in the range (0,Inf)
            }
            else {
                gaitFrequency = 1e8;  // Big enough to max out the gait frequency in any case
            }
        }

        // If desired, add basic timing feedback
        if (config["basicEnableTiming"].as<bool>() && config["basicGlobalEnable"].as<bool>()) {
            gaitFrequency += timingFreqDelta;
        }
    }

    // Coerce the gait frequency to the allowed range
    gaitFrequency = utility::math::clamp(1e-8, gaitFrequency, double(config["gaitFrequencyMax"].as<Expression>()));

    // Calculate the time to step based on how much gait phase we have to cover and how fast we intend to cover it
    double timeToStep = remainingGaitPhase / (M_PI * gaitFrequency);  // Note: Always in the range (0,Inf)

    // Update the gait phase with the appropriate phase increment
    double gaitPhaseIncrement = gaitFrequency * M_PI * systemIterationTime;
    // The gait phase must be in the range (-pi,pi], and thus must be wrapped!
    m_gaitPhase = utility::math::angle::normalizeAngle(m_gaitPhase + gaitPhaseIncrement);

    // Increment the blending phase if we are in the process of blending
    if (m_blending) {
        m_blendPhase += gaitPhaseIncrement;
    }

    //
    // Gait command vector update
    //

    // By default use a nominal CL step size (this should approximately emulate walking OL with zero GCV)
    Eigen::Vector3f stepSize(0.0, 2.0 * supportLegSign * config["hipWidth"].as<Expression>(), 0.0);

    // If desired, use the TX model step size instead
    if (config["cmdUseTXStepSize"].as<bool>()) {
        stepSize = txModel.stepSize;
    }

    // Clamp the step size to a maximal radius
    stepSize.x() = utility::math::clamp(-float(config["mgMaxStepRadiusX"].as<Expression>()),
                                        stepSize.x(),
                                        float(config["mgMaxStepRadiusX"].as<Expression>()));
    stepSize.y() = utility::math::clamp(-float(config["mgMaxStepRadiusY"].as<Expression>()),
                                        stepSize.y(),
                                        float(config["mgMaxStepRadiusY"].as<Expression>()));

    // Step size to GCV conversion.
    // To dodge the fact that the gait engine can only take symmetrical steps, while the limp model may produce
    // asymmetrical steps, we use the center point between the feet as the sex and map the sex position between alpha,
    // delta and omega to the gcv.
    Eigen::Vector3d gcvTarget;
    Eigen::Vector3f sex = 0.5 * stepSize;
    limp.set(0.0, sigma, C);
    limp.update(contrib::Limp(alpha, 0, C).tLoc(delta));
    gcvTarget.x() = sex.x() / limp.x0;
    if (supportLegSign * gcvInput.y() >= 0) {
        gcvTarget.y() = supportLegSign * utility::math::clamp(-1.0, (std::abs(sex.y()) - delta) / (omega - delta), 1.0);
    }
    else {
        gcvTarget.y() = oldGcvTargetY;  // TODO: This looks relatively unsafe
    }
    gcvTarget.z() = sex.z();
    oldGcvTargetY = gcvTarget.y();
    if (!config["cmdAllowCLStepSizeX"].as<bool>()) {
        gcvTarget.x() = 0.0;
    }

    // Move the current gcv smoothly to the target gcv in timeToStep amount of time
    Eigen::Vector3d CLGcv = gcvTarget;

    // TODO: Build some kind of protection into this update strategy to avoid increasingly explosive GCV updates
    // super-close to the end of the step, causing sharp joint curves.
    if (timeToStep > systemIterationTime) {
        CLGcv = m_gcv + (systemIterationTime / timeToStep) * (gcvTarget - m_gcv);
    }

    // Handle the situation differently depending on whether we are using CL step sizes or not
    // Closed loop step sizes...
    if (config["cmdUseCLStepSize"].as<bool>()) {
        // Set the calculated closed loop GCV
        if (noCLStepsCounter > 0) {
            m_gcv.setZero();
            noCLStepsCounter--;
        }
        else {
            if (config["cmdAllowCLStepSizeX"].as<bool>()) {
                m_gcv.x() = CLGcv.x();
            }
            else {
                double D = config["gcvDecToAccRatio"].as<bool>();
                if (gcvUnbiased.x() >= 0.0) {
                    m_gcv.x() += utility::math::clamp(-truedT * config["gcvAccForwards"].as<Expression>() * D,
                                                      m_gcvInput.x() - m_gcv.x(),
                                                      truedT * config["gcvAccForwards"].as<Expression>());
                }
                else {
                    m_gcv.x() += utility::math::clamp(-truedT * config["gcvAccBackwards"].as<Expression>(),
                                                      m_gcvInput.x() - m_gcv.x(),
                                                      truedT * config["gcvAccBackwards"].as<Expression>() * D);
                }
            }
            m_gcv.y() = CLGcv.y();
            m_gcv.z() = CLGcv.z();
        }
    }
    // Open loop step sizes...
    else {
        // Zero out the step size to show that we are OL
        stepSize = Eigen::Vector3f::Zero();

        // Update the internal gcv based on the gcv input using a slope-limiting approach
        double D = config["gcvDecToAccRatio"].as<Expression>();
        if (gcvUnbiased.x() >= 0.0) {
            m_gcv.x() += utility::math::clamp(-truedT * config["gcvAccForwards"].as<Expression>() * D,
                                              m_gcvInput.x() - m_gcv.x(),
                                              truedT * config["gcvAccForwards"].as<Expression>());
        }
        else {
            m_gcv.x() += utility::math::clamp(-truedT * config["gcvAccBackwards"].as<Expression>(),
                                              m_gcvInput.x() - m_gcv.x(),
                                              truedT * config["gcvAccBackwards"].as<Expression>() * D);
        }
        if (gcvUnbiased.y() >= 0.0) {
            m_gcv.y() += utility::math::clamp(-truedT * config["gcvAccSidewards"].as<Expression>() * D,
                                              m_gcvInput.y() - m_gcv.y(),
                                              truedT * config["gcvAccSidewards"].as<Expression>());
        }
        else {
            m_gcv.y() += utility::math::clamp(-truedT * config["gcvAccSidewards"].as<Expression>(),
                                              m_gcvInput.y() - m_gcv.y(),
                                              truedT * config["gcvAccSidewards"].as<Expression>() * D);
        }
        if (gcvUnbiased.z() >= 0.0) {
            m_gcv.z() += utility::math::clamp(-truedT * config["gcvAccRotational"].as<Expression>() * D,
                                              m_gcvInput.z() - m_gcv.z(),
                                              truedT * config["gcvAccRotational"].as<Expression>());
        }
        else {
            m_gcv.z() += utility::math::clamp(-truedT * config["gcvAccRotational"].as<Expression>(),
                                              m_gcvInput.z() - m_gcv.z(),
                                              truedT * config["gcvAccRotational"].as<Expression>() * D);
        }
    }

    // Calculate and filter the gait command acceleration
    m_gcvDeriv.put(m_gcv);
    Eigen::Vector3d gcvAccRaw = m_gcvDeriv.value() / systemIterationTime;
    m_gcvAccSmoothX.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.x(), m_gcvAcc.x(), config["gcvAccJerkLimitX"].as<Expression>() * systemIterationTime));
    m_gcvAccSmoothY.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.y(), m_gcvAcc.y(), config["gcvAccJerkLimitY"].as<Expression>() * systemIterationTime));
    m_gcvAccSmoothZ.put(utility::math::filter::SlopeLimiter::eval(
        gcvAccRaw.z(), m_gcvAcc.z(), config["gcvAccJerkLimitZ"].as<Expression>() * systemIterationTime));
    m_gcvAcc.x() = m_gcvAccSmoothX.value();
    m_gcvAcc.y() = m_gcvAccSmoothY.value();
    m_gcvAcc.z() = m_gcvAccSmoothZ.value();

    //
    // Motion stance control
    //

    // Calculate the current target motion stance factor
    double targetLegAngleXFact = 1.0;
    bool haveMotionStance      = (config["enableMotionStances"].as<Expression>() && in.motionPending);
    if (haveMotionStance) {
        switch (in.motionStance) {
            case STANCE_DEFAULT: targetLegAngleXFact = 1.0; break;
            case STANCE_KICK: targetLegAngleXFact    = 0.0; break;
            default: targetLegAngleXFact             = 1.0; break;
        }
    }

    // Update the motion stance factors, taking into consideration which legs we are allowed to adjust
    if (!haveMotionStance
        || (gcvUnbiased.norm() <= config["stanceAdjustGcvMax"].as<Expression>()
            && ((in.motionAdjustLeftFoot && rxRobotModel.supportLegSign == contrib::RobotModel::RIGHT_LEG)
                || (in.motionAdjustRightFoot && rxRobotModel.supportLegSign == contrib::RobotModel::LEFT_LEG)
                || (!in.motionAdjustLeftFoot && !in.motionAdjustRightFoot)))) {
        m_motionLegAngleXFact = utility::math::filter::SlopeLimiter::eval(
            targetLegAngleXFact,
            m_motionLegAngleXFact,
            config["stanceAdjustRate"].as<Expression>() * systemIterationTime);
    }

    // Determine whether the motion stance adjustment is still ongoing
    bool motionStanceOngoing = (std::abs(m_motionLegAngleXFact - targetLegAngleXFact) > 1e-6);

    //
    // Walking control
    //

    // Check whether we should stop walking
    double nominalGaitPhaseInc = M_PI * systemIterationTime * config["gaitFrequency"].as<Expression>();
    double LB                  = nominalGaitPhaseInc
                * config["stoppingPhaseTolLB"].as<Expression>();  // Lower bound tolerance config variable is in the
                                                                  // units of nominal phase increments, calculated
                                                                  // using nominalStepTime (also a config variable)
    double UB = nominalGaitPhaseInc
                * config["stoppingPhaseTolUB"].as<Expression>();  // Upper bound tolerance config variable is in the
                                                                  // units of nominal phase increments, calculated
                                                                  // using nominalStepTime (also a config variable)
    if (!m_walk && gcvUnbiased.norm() <= config["stoppingGcvMag"].as<Expression>()
        &&  // Note that this is intentionally checking the gcv (unbiased) from the last cycle, not the new and
        // unexecuted modified one from the current call to this update function
        ((m_gaitPhase >= 0.0 && oldGaitPhase <= 0.0 && (m_gaitPhase <= UB || oldGaitPhase >= -LB))
         || (m_gaitPhase <= 0.0 && oldGaitPhase >= 0.0 && (m_gaitPhase <= UB - M_PI || oldGaitPhase >= M_PI - LB)))
        && (!motionStanceOngoing)) {
        resetWalking(false, gcvBias);
    }

    //
    // Plotting
    //

    // Plot update function variables
    if (plot) {
        reactor.emit(graph("bonn/rxrmodel_suppvec",
                           rxRobotModel.suppComVector().x(),
                           rxRobotModel.suppComVector().y(),
                           rxRobotModel.suppComVector().z()));
        reactor.emit(graph("bonn/rxrmodel_stepvec",
                           rxRobotModel.suppStepVector().x(),
                           rxRobotModel.suppStepVector().y(),
                           rxRobotModel.suppStepVector().z()));
        reactor.emit(graph("bonn/rxrmodel_stepvec_fyaw", rxRobotModel.suppStepYaw()));
        reactor.emit(graph("bonn/fused_x", fusedX));
        reactor.emit(graph("bonn/fused_y", fusedY));
        reactor.emit(graph("bonn/comfilter_x", m_comFilter.x()));
        reactor.emit(graph("bonn/comfilter_y", m_comFilter.y()));
        reactor.emit(graph("bonn/comfilter_vx", m_comFilter.vx()));
        reactor.emit(graph("bonn/comfilter_vy", m_comFilter.vy()));
        reactor.emit(graph("bonn/rxmodel_x", rxModel.x));
        reactor.emit(graph("bonn/rxmodel_y", rxModel.y));
        reactor.emit(graph("bonn/rxmodel_vx", rxModel.vx));
        reactor.emit(graph("bonn/rxmodel_vy", rxModel.vy));
        reactor.emit(graph("bonn/rxmodel_suppleg", rxModel.supportLegSign));
        reactor.emit(graph("bonn/rxmodel_timetostep", rxModel.timeToStep));
        reactor.emit(graph("bonn/mxmodel_x", mxModel.x));
        reactor.emit(graph("bonn/mxmodel_y", mxModel.y));
        reactor.emit(graph("bonn/mxmodel_vx", mxModel.vx));
        reactor.emit(graph("bonn/mxmodel_vy", mxModel.vy));
        reactor.emit(graph("bonn/mxmodel_suppleg", mxModel.supportLegSign));
        reactor.emit(graph("bonn/mxmodel_timetostep", mxModel.timeToStep));
        reactor.emit(graph("bonn/mxmodel_zmp_x", mxModel.zmp.x()));
        reactor.emit(graph("bonn/mxmodel_zmp_y", mxModel.zmp.y()));
        reactor.emit(graph("bonn/txmodel_x", txModel.x));
        reactor.emit(graph("bonn/txmodel_y", txModel.y));
        reactor.emit(graph("bonn/txmodel_vx", txModel.vx));
        reactor.emit(graph("bonn/txmodel_vy", txModel.vy));
        reactor.emit(graph("bonn/txmodel_suppleg", txModel.supportLegSign));
        reactor.emit(graph("bonn/txmodel_timetostep", txModel.timeToStep));
        reactor.emit(graph("bonn/txmodel_stepsizex", txModel.stepSize.x()));
        reactor.emit(graph("bonn/txmodel_stepsizey", txModel.stepSize.y()));
        reactor.emit(graph("bonn/txmodel_stepsizez", txModel.stepSize.z()));
        reactor.emit(graph("bonn/adaptation_x", adaptx));
        reactor.emit(graph("bonn/adaptation_y", adapty));
        reactor.emit(graph("bonn/exp_fused_x", expectedFusedX));
        reactor.emit(graph("bonn/exp_fused_y", expectedFusedY));
        reactor.emit(graph("bonn/dev_fused_x", deviationFusedX));
        reactor.emit(graph("bonn/dev_fused_y", deviationFusedY));
        reactor.emit(graph("bonn/feedback_fused_x", fusedXFeed));
        reactor.emit(graph("bonn/feedback_fused_y", fusedYFeed));
        reactor.emit(graph("bonn/feedback_dfused_x", dFusedXFeed));
        reactor.emit(graph("bonn/feedback_dfused_y", dFusedYFeed));
        reactor.emit(graph("bonn/feedback_ifused_x", iFusedXFeed));
        reactor.emit(graph("bonn/feedback_ifused_y", iFusedYFeed));
        reactor.emit(graph("bonn/feedback_gyro_x", gyroXFeed));
        reactor.emit(graph("bonn/feedback_gyro_y", gyroYFeed));
        reactor.emit(graph("bonn/timing_feed_weight", timingFeedWeight));
        reactor.emit(graph("bonn/timing_freq_delta", timingFreqDelta));
        reactor.emit(graph("bonn/gait_frequency", gaitFrequency));
        reactor.emit(graph("bonn/rem_gait_phase", remainingGaitPhase));
        reactor.emit(graph("bonn/timetostep", timeToStep));
        reactor.emit(graph("bonn/stepsize_x", stepSize.x()));
        reactor.emit(graph("bonn/stepsize_y", stepSize.y()));
        reactor.emit(graph("bonn/stepsize_z", stepSize.z()));
        reactor.emit(graph("bonn/gcvtarget_x", gcvTarget.x()));
        reactor.emit(graph("bonn/gcvtarget_y", gcvTarget.y()));
        reactor.emit(graph("bonn/gcvtarget_z", gcvTarget.z()));
        reactor.emit(graph("bonn/last_step_duration", lastStepDuration));
    }
}


// Calculate common motion data
GaitEngine::CommonMotionData GaitEngine::calcCommonMotionData(bool isFirst) const {
    // Declare variables
    CommonMotionData CMD;

    // Set the gcv and absolute gcv
    CMD.gcvX = config["gcvPrescalerLinVelX"].as<Expression>()
               * m_gcv.x();  // The raw commanded gcv x prescaled by a preconfigured scaler
                             // to adjust the dynamic range of the gait for inputs in the
                             // range [0,1]
    CMD.gcvY = config["gcvPrescalerLinVelY"].as<Expression>()
               * m_gcv.y();  // The raw commanded gcv y prescaled by a preconfigured scaler
                             // to adjust the dynamic range of the gait for inputs in the
                             // range [0,1]
    CMD.gcvZ = config["gcvPrescalerAngVelZ"].as<Expression>()
               * m_gcv.z();            // The raw commanded gcv z prescaled by a preconfigured scaler
                                       // to adjust the dynamic range of the gait for inputs in the
                                       // range [0,1]
    CMD.absGcvX = std::abs(CMD.gcvX);  // Absolute commanded gcv x
    CMD.absGcvY = std::abs(CMD.gcvY);  // Absolute commanded gcv y
    CMD.absGcvZ = std::abs(CMD.gcvZ);  // Absolute commanded gcv z

    // Set the gait phase variables
    CMD.gaitPhase = m_gaitPhase;  // gaitPhase: Starts at 0 at the beginning of walking, and has the swing phase of the
                                  // first leg in [0,pi] (left or right depending on m_leftLegFirst)
    CMD.oppGaitPhase = utility::math::angle::normalizeAngle(
        CMD.gaitPhase + M_PI);  // oppGaitPhase: Is in exact antiphase to gaitPhase, and has the
                                // support phase of the first leg in [0,pi] (left or right
                                // depending on m_leftLegFirst)
    CMD.limbPhase = (isFirst ? CMD.gaitPhase : CMD.oppGaitPhase);  // limbPhase: Is in exact phase or antiphase to
                                                                   // m_gaitPhase, and always has the swing phase of
                                                                   // 'this' limb in [0,pi]
    CMD.absPhase = (m_leftLegFirst ? CMD.gaitPhase : CMD.oppGaitPhase);  // absPhase: Is in exact phase or antiphase to
                                                                         // m_gaitPhase, and always has the swing phase
                                                                         // of the left leg in [0,pi]

    // Set the first collection of phase marks
    CMD.doubleSupportPhase =
        config["doubleSupportPhaseLen"].as<Expression>();  // The length of the double support phase (from gait phase
                                                           // zero to this value), in the range [0,pi/2]
    CMD.swingStartPhase = CMD.doubleSupportPhase
                          + config["swingStartPhaseOffset"]
                                .as<Expression>();  // Limb phase mark at which the swing starts, in the range [0,pi]
    CMD.swingStopPhase = M_PI
                         - config["swingStopPhaseOffset"]
                               .as<Expression>();  // Limb phase mark at which the swing stops, in the range [0,pi]

    // Check for violation of the minimum allowed swing phase length and automatically rescale the swing phase
    // offsets
    // if required
    if (CMD.swingStopPhase - CMD.swingStartPhase
        < config["swingMinPhaseLen"].as<Expression>())  // The config parameter swingMinPhaseLen
    // specifies the minimum allowed value of
    // sinusoidPhaseLen, in the range [0.1,1]
    {
        double resize = (M_PI - CMD.doubleSupportPhase - config["swingMinPhaseLen"].as<Expression>())
                        / (config["swingStartPhaseOffset"].as<Expression>()
                           + config["swingStopPhaseOffset"].as<Expression>());  // The denominator of this should never
                                                                                // be <= 0 due to the permitted ranges
                                                                                // on the various config variables
        CMD.swingStartPhase = CMD.doubleSupportPhase + resize * config["swingStartPhaseOffset"].as<Expression>();
        CMD.swingStopPhase  = M_PI - resize * config["swingStopPhaseOffset"].as<Expression>();
        NUClear::log(3.0,
                     "Invalid configuration for cap_gait swing phase (in violation of minimum swing phase length) "
                     "=> Automatically fixing");
    }

    // Set the remaining phase marks
    CMD.suppTransStartPhase = -config["suppTransStartRatio"].as<Expression>()
                              * (M_PI - CMD.swingStopPhase);  // Phase mark at which the support transition
                                                              // starts (small negative number, i.e. backwards
                                                              // from zero/pi, the ratio interpolates between
                                                              // the beginning of the double support phase
    // (zero) and the first negative swing stop phase)
    CMD.suppTransStopPhase =
        CMD.doubleSupportPhase
        + config["suppTransStopRatio"].as<Expression>()
              * (CMD.swingStartPhase - CMD.doubleSupportPhase);  // Phase mark at which the support transition stops, in
                                                                 // the range [0,pi] (the ratio interpolates between the
                                                                 // end of the double support phase and the first
                                                                 // positive swing start phase)

    // Calculate the extra swing variables
    CMD.liftingPhaseLen = M_PI - CMD.doubleSupportPhase;  // Length of each lifting (single support) phase
    CMD.suppPhaseLen = M_PI - CMD.suppTransStartPhase + CMD.suppTransStopPhase;  // Length of the support phase (phase
                                                                                 // length for which a support
                                                                                 // coefficient is non-zero at a time)
    CMD.nonSuppPhaseLen = (2.0 * M_PI) - CMD.suppPhaseLen;  // Length of the non-support phase (phase length for which a
                                                            // support coefficient is zero at a time)
    CMD.sinusoidPhaseLen = CMD.swingStopPhase - CMD.swingStartPhase;  // Length of the sinusoidal forwards swing phase
    CMD.linearPhaseLen   = (2.0 * M_PI) - CMD.sinusoidPhaseLen;       // Length of the linear backwards swing phase

    // Calculate the gait phase dependent dimensionless swing angle (ranging from -1 to 1)
    if (CMD.limbPhase >= CMD.swingStartPhase && CMD.limbPhase <= CMD.swingStopPhase) {
        CMD.swingAngle =
            -std::cos(M_PI * (CMD.limbPhase - CMD.swingStartPhase)
                      / CMD.sinusoidPhaseLen);  // Sinusoid forwards swing from dimensionless angle -1 to +1
        // (section from phase swingStartPhase to phase swingStopPhase)
    }
    else if (CMD.limbPhase > CMD.swingStopPhase) {
        CMD.swingAngle =
            1.0 - (2.0 / CMD.linearPhaseLen) * (CMD.limbPhase - CMD.swingStopPhase);  // Linear backwards swing from
                                                                                      // dimensionless angle +1 to a
        // mid-way point C (section from
        // phase swingStopPhase to phase
        // pi)
    }
    else {
        CMD.swingAngle = 1.0
                         - (2.0 / CMD.linearPhaseLen)
                               * (CMD.limbPhase - CMD.swingStopPhase + (2.0 * M_PI));  // Linear backwards swing
                                                                                       // from dimensionless
                                                                                       // angle C to -1 (section
                                                                                       // from phase -pi to phase
                                                                                       // swingStartPhase)
    }

    // Return the common motion data
    return CMD;
}

// Generate the abstract leg motion
// 'leg' is assumed to contain the desired leg halt pose
void GaitEngine::abstractLegMotion(pose::AbstractLegPose& leg) {
    //
    // Common motion data
    //

    // The legs swing about all three axes to achieve the required walking velocity. They do this with a special
    // timing
    // and waveform, summarised by the dimensionless swing angle waveform template. This template consists of a
    // sinusoidal 'forwards' motion, followed by a linear 'backwards' motion. The forwards motion is referred to as
    // the
    // swing phase of the corresponding limb, while the backwards motion is referred to as the support phase. This
    // alludes to the expected behaviour that the foot of a particular leg should be in contact with the ground
    // during
    // that leg's support phase, and in the air during its swing phase. The ratio of swing time to support time in a
    // human gait is typically 40 to 60, and so the swing phase is made shorter. The exact timing of the swing phase
    // is
    // defined by the swingStartPhaseOffset, swingStopPhaseOffset and doubleSupportPhaseLen configuration variables.
    // It
    // is expected that the foot achieves toe-off at approximately the swing start phase, and heel-strike at
    // approximately the swing stop phase. The asymmetry of the two phases in combination with the explicit double
    // support phase leads to a brief-ish phase where both legs are expected to be in contact with the ground
    // simultaneously, performing their backwards swing.

    // Calculate the common motion data
    CommonMotionData CMD = calcCommonMotionData(leg.cld.isLeft == m_leftLegFirst);

    // Disable the leg swing if required
    if (config["tuningNoLegSwing"].as<bool>()) {
        CMD.swingAngle = 0.0;
    }

    // Initialise the additive hip angles
    double hipAngleX = 0.0;
    double hipAngleY = 0.0;

    //
    // Leg lifting (extension, angleY)
    //

    // The leg is alternately lifted off the ground (step) and pushed down into it (push), with a short break in-between
    // given by the double support phase. The double support phases start at limb phase 0 and pi, and the leg
    // lifting/pushing phases start immediately after the double support phases end, respectively. The lifting/pushing
    // phases always end at limb phases of pi and 0, respectively. The leg push phase, if enabled, assists with
    // achieving foot clearance of the swing foot.

    // Apply leg stepping and pushing to the abstract leg pose
    double legExtensionOffset = 0.0;
    if (!config["tuningNoLegLifting"].as<bool>()) {
        // Calculate the desired step and push height from the current gcv
        double stepHeight = config["legStepHeight"].as<Expression>()
                            + CMD.absGcvX * config["legStepHeightGradX"].as<Expression>()
                            + CMD.absGcvY * config["legStepHeightGradY"].as<Expression>();
        double pushHeight =
            config["legPushHeight"].as<Expression>() + CMD.absGcvX * config["legPushHeightGradX"].as<Expression>();

        // Precalculate parameters and phases for the following calculation
        double sinAngFreq     = M_PI / CMD.liftingPhaseLen;
        double stepStartPhase = CMD.limbPhase - CMD.doubleSupportPhase;
        double stepStopPhase  = -utility::math::angle::normalizeAngle(CMD.limbPhase - M_PI);
        double pushStartPhase = stepStartPhase + M_PI;
        double pushStopPhase  = -CMD.limbPhase;

        // Calculate the basic sinusoid step and push waveforms
        if (stepStartPhase >= 0.0) {
            legExtensionOffset =
                stepHeight
                * std::sin(sinAngFreq * stepStartPhase);  // Leg stepping phase => Limb phase in the positive half
        }
        else if (pushStartPhase >= 0.0 && CMD.limbPhase <= 0.0) {
            legExtensionOffset =
                -pushHeight
                * std::sin(sinAngFreq * pushStartPhase);  // Leg pushing phase => Limb phase in the negative half
        }
        else {
            legExtensionOffset = 0.0;  // Double support phase
        }

        // Add fillets to the waveforms to avoid overly large acceleration/torque jumps
        legExtensionOffset +=
            utility::math::filter::LinSinFillet::eval(stepStartPhase,
                                                      stepHeight,
                                                      sinAngFreq,
                                                      0.5 * config["filletStepPhaseLen"].as<Expression>(),
                                                      config["doubleSupportPhaseLen"].as<Expression>());
        legExtensionOffset +=
            utility::math::filter::LinSinFillet::eval(stepStopPhase,
                                                      stepHeight,
                                                      sinAngFreq,
                                                      0.5 * config["filletStepPhaseLen"].as<Expression>(),
                                                      config["doubleSupportPhaseLen"].as<Expression>());
        legExtensionOffset +=
            utility::math::filter::LinSinFillet::eval(pushStartPhase,
                                                      -pushHeight,
                                                      sinAngFreq,
                                                      0.5 * config["filletPushPhaseLen"].as<Expression>(),
                                                      config["doubleSupportPhaseLen"].as<Expression>());
        legExtensionOffset +=
            utility::math::filter::LinSinFillet::eval(pushStopPhase,
                                                      -pushHeight,
                                                      sinAngFreq,
                                                      0.5 * config["filletPushPhaseLen"].as<Expression>(),
                                                      config["doubleSupportPhaseLen"].as<Expression>());

        // Update the leg extension
        leg.extension += legExtensionOffset;
        leg.angleY += config["legExtToAngleYGain"].as<Expression>()
                      * legExtensionOffset;  // This trims the lift angle of the feet, which
                                             // can be used to trim walking on the spot, but
                                             // this also has very positive effects on the
                                             // remainder of OL walking
    }

    //
    // Sagittal leg swing (angleY)
    //

    // Apply the sagittal leg swing to the abstract leg pose (responsible for fulfilling the gcv x-velocity)
    // We use a different sagittal leg swing gradient depending on whether we're walking forwards or backwards.
    double legSagSwingMag = CMD.gcvX * (CMD.gcvX >= 0.0 ? config["legSagSwingMagGradXFwd"].as<Expression>()
                                                        : config["legSagSwingMagGradXBwd"].as<Expression>());
    double legSagSwing = -CMD.swingAngle * legSagSwingMag;
    leg.angleY += legSagSwing;

    //
    // Lateral leg pushout, lateral leg swing and lateral hip swing (angleX)
    //

    // Apply the lateral leg pushout to the abstract leg pose
    // This rolls the legs outwards (from the hip) in proportion to each of the absolute gcv values. This acts to
    // separate the feet more the faster the robot is walking, and the more the robot is trying to walk with combined
    // velocity components (e.g. forwards velocity coupled with a rotational velocity). This term seeks to prevent foot
    // to foot self-collisions, and should more be seen as a bias to the halt pose, as opposed to a dynamic motion
    // component of the gait. This is because for constant walking velocity command, this term is constant.
    double legLatPushoutMag = CMD.absGcvX * config["legLatPushoutMagGradX"].as<Expression>()
                              + CMD.absGcvY * config["legLatPushoutMagGradY"].as<Expression>()
                              + CMD.absGcvZ * config["legLatPushoutMagGradZ"].as<Expression>();
    if (!config["tuningNoLegPushout"].as<bool>()) {
        leg.angleX += leg.cld.limbSign * legLatPushoutMag;
    }

    // Apply the lateral leg swing to the abstract leg pose
    // This is responsible for fulfilling the gcv y-velocity.
    double legLatSwingMag = CMD.gcvY * config["legLatSwingMagGradY"].as<Expression>();
    double legLatSwing    = CMD.swingAngle * legLatSwingMag;
    leg.angleX += legLatSwing;

    // The lateral hip swing motion component sways the hips left and right relative to the feet. This is achieved via
    // direct addition of a hip swing term to the hip roll (i.e. angleX). Hip swing to the left occurs when the left
    // foot is in its support phase (i.e. absolute phase in (-pi,0]), and should correspond to negative roll. Hip swing
    // to the right occurs when the right foot is in its support phase (i.e. absolute phase in (0,pi]), and should
    // correspond to positive roll. The hip swing is applied equally to both hip roll joints, and is calculated as the
    // sum of two overlapping sinusoid 'halves', one for the hip swing to the left, and one for the hip swing to the
    // right. The overlap between the sinusoids occurs during the support transition phase, during which time the
    // sinusoids sum up, and the hip travels from one body side to the other. To be totally explicit, the left hip swing
    // sinusoid completes exactly half a sinusoid from the beginning of the left leg support transition to the end of
    // the next right leg support transition, and is zero everywhere else. The right hip swing sinusoid behaves
    // similarly, and they are added up to get the full dimensionless hip swing waveform. Recall that the absolute phase
    // is in phase or antiphase to the gait phase such that the swing phase of the left leg is in [0,pi].

    // Perform phase calculations for the left and right halves (sinusoids) of the hip swing motion
    double hipSwingStartL = CMD.suppTransStartPhase + M_PI;  // The start phase of the period of support of the left leg
    double hipSwingStartR = CMD.suppTransStartPhase;  // The start phase of the period of support of the right leg
    double hipSwingPhaseL = (CMD.absPhase < hipSwingStartL ? CMD.absPhase + (2.0 * M_PI) : CMD.absPhase)
                            - hipSwingStartL;  // The current phase relative to the start of the transition to the left
                                               // leg as the support leg, in the range [0,2*pi]
    double hipSwingPhaseR = (CMD.absPhase < hipSwingStartR ? CMD.absPhase + (2.0 * M_PI) : CMD.absPhase)
                            - hipSwingStartR;  // The current phase relative to the start of the transition to the right
                                               // leg as the support leg, in the range [0,2*pi]

    // Calculate the dimensionless hip swing angle (range -1 to 1) by summing up the two zero-clamped sinusoid
    // sub-waveforms
    double hipSwingAngleL = -std::max(std::sin(M_PI * hipSwingPhaseL / CMD.suppPhaseLen), 0.0);
    double hipSwingAngleR = std::max(std::sin(M_PI * hipSwingPhaseR / CMD.suppPhaseLen), 0.0);
    double hipSwingAngle  = hipSwingAngleL + hipSwingAngleR;  // The hip swing angle is in the range -1 to 1

    // Apply the lateral hip swing to the abstract leg pose
    double legLatHipSwingMag = config["legLatHipSwingMag"].as<Expression>()
                               + CMD.absGcvX * config["legLatHipSwingMagGradX"].as<Expression>()
                               + CMD.absGcvY * config["legLatHipSwingMagGradY"].as<Expression>();
    double legLatHipSwing = config["legLatHipSwingBias"].as<Expression>() + legLatHipSwingMag * hipSwingAngle;
    if (!config["tuningNoLegHipSwing"].as<Expression>()) {
        leg.angleX += legLatHipSwing;
    }

    //
    // Rotational leg V pushout and rotational leg swing (angleZ)
    //

    // Apply the rotational leg V pushout to the abstract leg pose
    // Offset the yaw of the feet to be in a tendentially more toe-out configuration the faster we are expecting the
    // robot to turn. This is referred to as rotational V pushout, and has the effect of attempting to avoid toe to toe
    // self-collisions. For a constant walking velocity command, this term is constant, and hence should be considered
    // to be more of a bias to the halt pose, rather than a dynamic motion component of the gait.
    double legRotVPushoutMag = CMD.absGcvZ * config["legRotVPushoutMagGradZ"].as<Expression>();
    if (!config["tuningNoLegPushout"].as<bool>()) {
        leg.angleZ += leg.cld.limbSign * legRotVPushoutMag;
    }

    // Apply the rotational leg swing to the abstract leg pose
    // This is responsible for fulfilling the gcv z-velocity.
    double legRotSwingMag = CMD.gcvZ * config["legRotSwingMagGradZ"].as<Expression>();
    double legRotSwing    = CMD.swingAngle * legRotSwingMag;
    leg.angleZ += legRotSwing;

    //
    // Leaning (hipAngleX, hipAngleY)
    //

    // Apply the leaning to the abstract leg pose
    // When walking at higher speeds, it is advantageous to make use of leaning to assist with balance. Velocity-based
    // leaning is chosen over acceleration-based leaning in the interest of producing continuous abstract pose outputs.
    double legSagLean = 0.0;
    double legLatLean = 0.0;
    if (!config["tuningNoLegLeaning"].as<bool>()) {
        // Lean forwards and backwards based on the sagittal walking velocity
        legSagLean = CMD.gcvX * (CMD.gcvX >= 0.0 ? config["legSagLeanGradVelXFwd"].as<Expression>()
                                                 : config["legSagLeanGradVelXBwd"].as<Expression>())
                     + CMD.absGcvZ * config["legSagLeanGradVelZAbs"].as<Expression>();
        legSagLean += m_gcvAcc.x() * (m_gcvAcc.x() >= 0.0 ? config["legSagLeanGradAccXFwd"].as<Expression>()
                                                          : config["legSagLeanGradAccXBwd"].as<Expression>());
        hipAngleY += -legSagLean;

        // Lean laterally into curves (based on the rotational walking velocity)
        legLatLean = CMD.gcvX * CMD.gcvZ * (CMD.gcvX >= 0.0 ? config["legLatLeanGradXZFwd"].as<Expression>()
                                                            : config["legLatLeanGradXZBwd"].as<Expression>());
        hipAngleX += legLatLean;
    }

    //
    // Basic feedback mechanisms (hipAngleX, hipAngleY, footAngleX, footAngleY)
    //

    // Apply fused angle and gyro feedback to the legs
    double hipAngleXFeedback     = 0.0;
    double hipAngleYFeedback     = 0.0;
    double footAngleXFeedback    = 0.0;
    double footAngleYFeedback    = 0.0;
    double footAngleCtsXFeedback = 0.0;
    double footAngleCtsYFeedback = 0.0;
    if (!config["tuningNoLegFeedback"].as<bool>()) {
        // Compute the phase waveform over which to apply the foot feedback
        double footPhaseLen =
            utility::math::clamp(0.05, double(config["basicFootAnglePhaseLen"].as<Expression>()), M_PI_2);
        double footFeedbackSlope =
            1.0 / footPhaseLen;  // This can't go uncontrolled because the footPhaseLen is clamped to a reasonable range
        double footFeedbackScaler = 0.0;
        // This works because of the clamp above
        if (CMD.limbPhase <= -M_PI_2) {
            footFeedbackScaler = utility::math::clamp(0.0, footFeedbackSlope * (CMD.limbPhase + M_PI), 1.0);
        }
        else {
            footFeedbackScaler =
                utility::math::clamp(0.0, footFeedbackSlope * (CMD.doubleSupportPhase - CMD.limbPhase), 1.0);
        }

        // Compute the integrated foot and hip angle feedback
        double hipAngleXIFeed     = config["basicIFusedHipAngleX"].as<Expression>() * iFusedXFeed;
        double hipAngleYIFeed     = config["basicIFusedHipAngleY"].as<Expression>() * iFusedYFeed;
        double footAngleCtsXIFeed = config["basicIFusedFootAngleCX"].as<Expression>() * iFusedXFeed;
        double footAngleCtsYIFeed = config["basicIFusedFootAngleCY"].as<Expression>() * iFusedYFeed;

        // Compute the total foot and hip angle feedback
        hipAngleXFeedback = config["basicFeedBiasHipAngleX"].as<Expression>()
                            + config["basicFusedHipAngleX"].as<Expression>() * fusedXFeed
                            + config["basicDFusedHipAngleX"].as<Expression>() * dFusedXFeed + hipAngleXIFeed
                            + config["basicGyroHipAngleX"].as<Expression>() * gyroXFeed;
        hipAngleYFeedback = config["basicFeedBiasHipAngleY"].as<Expression>()
                            + config["basicFusedHipAngleY"].as<Expression>() * fusedYFeed
                            + config["basicDFusedHipAngleY"].as<Expression>() * dFusedYFeed + hipAngleYIFeed
                            + config["basicGyroHipAngleY"].as<Expression>() * gyroYFeed;
        footAngleXFeedback = config["basicFeedBiasFootAngleX"].as<Expression>()
                             + config["basicFusedFootAngleX"].as<Expression>() * fusedXFeed
                             + config["basicDFusedFootAngleX"].as<Expression>() * dFusedXFeed
                             + config["basicIFusedFootAngleX"].as<Expression>() * iFusedXFeed
                             + config["basicGyroFootAngleX"].as<Expression>() * gyroXFeed;
        footAngleYFeedback = config["basicFeedBiasFootAngleY"].as<Expression>()
                             + config["basicFusedFootAngleY"].as<Expression>() * fusedYFeed
                             + config["basicDFusedFootAngleY"].as<Expression>() * dFusedYFeed
                             + config["basicIFusedFootAngleY"].as<Expression>() * iFusedYFeed
                             + config["basicGyroFootAngleY"].as<Expression>() * gyroYFeed;
        footAngleXFeedback *= footFeedbackScaler;
        footAngleYFeedback *= footFeedbackScaler;
        footAngleCtsXFeedback = config["basicFeedBiasFootAngCX"].as<Expression>() + footAngleCtsXIFeed;
        footAngleCtsYFeedback = config["basicFeedBiasFootAngCY"].as<Expression>() + footAngleCtsYIFeed;

        // Disable the foot and hip angle feedback if required
        if (!(config["basicEnableHipAngleX"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            hipAngleXFeedback = hipAngleXIFeed = 0.0;
        }
        if (!(config["basicEnableHipAngleY"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            hipAngleYFeedback = hipAngleYIFeed = 0.0;
        }
        if (!(config["basicEnableFootAngleX"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            footAngleXFeedback = 0.0;
        }
        if (!(config["basicEnableFootAngleY"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            footAngleYFeedback = 0.0;
        }
        if (!(config["basicEnableFootAngleCX"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            footAngleCtsXFeedback = footAngleCtsXIFeed = 0.0;
        }
        if (!(config["basicEnableFootAngleCY"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            footAngleCtsYFeedback = footAngleCtsYIFeed = 0.0;
        }

        // Apply the foot and hip angle feedback
        hipAngleX += hipAngleXFeedback;
        hipAngleY += hipAngleYFeedback;
        leg.footAngleX += footAngleXFeedback + footAngleCtsXFeedback;
        leg.footAngleY += footAngleYFeedback + footAngleCtsYFeedback;

        // Handle saving of the current integral feedback values as halt pose offsets
        if (m_saveIFeedToHaltPose && !m_savedLegIFeed) {
            // Note: The calculations of these offsets should match up with how the hip angle is added to the
            // abstract
            // pose further down in this function!
            config["haltLegAngleXBias"].as<Expression>() =
                config["haltLegAngleXBias"].as<Expression>() + hipAngleXIFeed;
            config["haltLegAngleY"].as<Expression>() = config["haltLegAngleY"].as<Expression>() + hipAngleYIFeed;
            config["haltFootAngleXBias"].as<Expression>() =
                config["haltFootAngleXBias"].as<Expression>() + footAngleCtsXIFeed + hipAngleXIFeed;
            config["haltFootAngleY"].as<Expression>() =
                config["haltFootAngleY"].as<Expression>() + footAngleCtsYIFeed + hipAngleYIFeed;
            config["haltLegExtensionBias"].as<Expression>() =
                config["haltLegExtensionBias"].as<Expression>()
                + config["legHipAngleXLegExtGain"].as<Expression>() * std::sin(hipAngleXIFeed);
            NUClear::log("Saved the current integrated leg feedback offsets as modifications to the halt pose");
            m_savedLegIFeed = true;
        }

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config["basicGlobalEnable"].as<bool>()
            && ((config["basicIFusedHipAngleX"].as<Expression>() != 0.0 && config["basicEnableHipAngleX"].as<bool>())
                || (config["basicIFusedFootAngleX"].as<Expression>() != 0.0
                    && config["basicEnableFootAngleX"].as<bool>())
                || (config["basicIFusedFootAngleCX"].as<Expression>() != 0.0
                    && config["basicEnableFootAngleCX"].as<bool>()))) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config["basicGlobalEnable"].as<bool>()
            && ((config["basicIFusedHipAngleY"].as<Expression>() != 0.0 && config["basicEnableHipAngleY"].as<bool>())
                || (config["basicIFusedFootAngleY"].as<Expression>() != 0.0
                    && config["basicEnableFootAngleY"].as<bool>())
                || (config["basicIFusedFootAngleCY"].as<Expression>() != 0.0
                    && config["basicEnableFootAngleCY"].as<bool>()))) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Hip angle (angleX, angleY, footAngleX, footAngleY, extension)
    //

    // Add the required hip angle X to the abstract pose
    leg.angleX += hipAngleX;
    leg.footAngleX += hipAngleX;
    if (utility::math::sign0(hipAngleX) == leg.cld.limbSign) {
        leg.extension += config["legHipAngleXLegExtGain"].as<Expression>() * std::abs(std::sin(hipAngleX));
    }

    // Add the required hip angle Y to the abstract pose
    leg.angleY += hipAngleY;
    leg.footAngleY += hipAngleY;

    //
    // Support coefficients
    //

    // The support coefficient gives the proportion of the robot's weight that is expected to be on this leg at the
    // current time. It is a dimensionless parameter in the range from 0 to 1, and ideally the sum of the left and right
    // leg support coefficients should be 1 at all times. In general during a leg's swing phase the support coefficient
    // is 0, during the support phase the support coefficient is 1, and over the double support phase the support
    // coefficient is linearly blended. The exact start and stop of the support transition blends are given by
    // CMD.suppTransStartPhase and CMD.suppTransStopPhase.

    // Calculate the phase length over which the support coefficient transition should take place
    double supportTransitionPhaseLen = CMD.suppTransStopPhase - CMD.suppTransStartPhase;
    double supportTransitionSlope    = 1e10;
    if (supportTransitionPhaseLen > 1e-10) {
        supportTransitionSlope = 1.0 / supportTransitionPhaseLen;
    }

    // Calculate the required support coefficient of this leg
    if (CMD.limbPhase <= CMD.suppTransStopPhase - M_PI) {
        leg.cld.supportCoeff = utility::math::clamp(
            0.0, 1.0 - supportTransitionSlope * (CMD.suppTransStopPhase - M_PI - CMD.limbPhase), 1.0);
    }
    else if (CMD.limbPhase <= CMD.suppTransStartPhase) {
        leg.cld.supportCoeff = 1.0;
    }
    else if (CMD.limbPhase <= CMD.suppTransStopPhase) {
        leg.cld.supportCoeff =
            utility::math::clamp(0.0, supportTransitionSlope * (CMD.suppTransStopPhase - CMD.limbPhase), 1.0);
    }
    else if (CMD.limbPhase >= CMD.suppTransStartPhase + M_PI) {
        leg.cld.supportCoeff =
            utility::math::clamp(0.0, supportTransitionSlope * (CMD.limbPhase - CMD.suppTransStartPhase - M_PI), 1.0);
    }
    else {
        leg.cld.supportCoeff = 0.0;
    }

    // Rescale the support coefficients to the required range
    leg.cld.supportCoeff = config["supportCoeffRange"].as<Expression>() * (leg.cld.supportCoeff - 0.5) + 0.5;

    // Disable variations in the support coefficients if required
    if (config["tuningNoLegSuppCoeff"].as<bool>()) {
        leg.cld.supportCoeff = 0.5;
    }

    //
    // Plotting
    //

    // Plot the leg motion components
    if (plot) {
        // clang-format off
        reactor.emit(graph(std::string("bonn/leg_extension_")        + (leg.cld.isLeft ? "L" : "R"), legExtensionOffset));
        reactor.emit(graph(std::string("bonn/leg_swing_angle_")      + (leg.cld.isLeft ? "L" : "R"), CMD.swingAngle));
        reactor.emit(graph(std::string("bonn/leg_sag_swing_")        + (leg.cld.isLeft ? "L" : "R"), legSagSwing));
        reactor.emit(graph(std::string("bonn/leg_lat_swing_")        + (leg.cld.isLeft ? "L" : "R"), legLatSwing));
        reactor.emit(graph(std::string("bonn/leg_rot_swing_")        + (leg.cld.isLeft ? "L" : "R"), legRotSwing));
        reactor.emit(graph(std::string("bonn/leg_lat_hip_swing_")    + (leg.cld.isLeft ? "L" : "R"), legLatHipSwing));
        reactor.emit(graph(std::string("bonn/leg_sag_lean_")         + (leg.cld.isLeft ? "L" : "R"), legSagLean));
        reactor.emit(graph(std::string("bonn/leg_lat_lean_")         + (leg.cld.isLeft ? "L" : "R"), legLatLean));
        reactor.emit(graph(std::string("bonn/leg_feed_hipanglex_")   + (leg.cld.isLeft ? "L" : "R"), hipAngleXFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_hipangley_")   + (leg.cld.isLeft ? "L" : "R"), hipAngleYFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_footanglex_")  + (leg.cld.isLeft ? "L" : "R"), footAngleXFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_footangley_")  + (leg.cld.isLeft ? "L" : "R"), footAngleYFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_footanglecx_") + (leg.cld.isLeft ? "L" : "R"), footAngleCtsXFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_footanglecy_") + (leg.cld.isLeft ? "L" : "R"), footAngleCtsYFeedback));
        reactor.emit(graph(std::string("bonn/leg_abs_legext_")       + (leg.cld.isLeft ? "L" : "R"), leg.extension));
        reactor.emit(graph(std::string("bonn/leg_abs_leganglex_")    + (leg.cld.isLeft ? "L" : "R"), leg.angleX));
        reactor.emit(graph(std::string("bonn/leg_abs_legangley_")    + (leg.cld.isLeft ? "L" : "R"), leg.angleY));
        reactor.emit(graph(std::string("bonn/leg_abs_leganglez_")    + (leg.cld.isLeft ? "L" : "R"), leg.angleZ));
        reactor.emit(graph(std::string("bonn/leg_abs_footanglex_")   + (leg.cld.isLeft ? "L" : "R"), leg.footAngleX));
        reactor.emit(graph(std::string("bonn/leg_abs_footangley_")   + (leg.cld.isLeft ? "L" : "R"), leg.footAngleY));
        // clang-format on
    }
}

// Generate the abstract arm motion
// 'arm' is assumed to contain the desired arm halt pose
void GaitEngine::abstractArmMotion(pose::AbstractArmPose& arm) {
    //
    // Common motion data
    //

    // The limb phase and swing angles for the arms are calculated exactly like for the legs, only the limb phase is
    // inverted. That is, the support and swing phases of the left arm match those of the right leg, and vice versa.

    // Calculate the common motion data
    CommonMotionData CMD = calcCommonMotionData(arm.cad.isLeft != m_leftLegFirst);

    // Disable the arm swing if required
    if (config["tuningNoArmSwing"].as<bool>()) {
        CMD.swingAngle = 0.0;
    }

    //
    // Sagittal arm swing (angleY)
    //

    // Apply the sagittal arm swing to the abstract arm pose
    double armSagSwingMag =
        config["armSagSwingMag"].as<Expression>() + CMD.gcvX * config["armSagSwingMagGradX"].as<Expression>();
    double armSagSwing = -CMD.swingAngle * armSagSwingMag;
    arm.angleY += armSagSwing;

    //
    // Basic feedback mechanisms (angleX, angleY)
    //

    // Apply fused angle and gyro feedback to the arms
    double armAngleXFeedback = 0.0;
    double armAngleYFeedback = 0.0;
    if (!config["tuningNoArmFeedback"].as<bool>()) {
        // Compute the integrated arm angle feedback
        double armAngleXIFeed = config["basicIFusedArmAngleX"].as<Expression>() * iFusedXFeed;
        double armAngleYIFeed = config["basicIFusedArmAngleY"].as<Expression>() * iFusedYFeed;

        // Compute the total arm angle feedback
        armAngleXFeedback = config["basicFeedBiasArmAngleX"].as<Expression>()
                            + config["basicFusedArmAngleX"].as<Expression>() * fusedXFeed
                            + config["basicDFusedArmAngleX"].as<Expression>() * dFusedXFeed + armAngleXIFeed
                            + config["basicGyroArmAngleX"].as<Expression>() * gyroXFeed;
        armAngleYFeedback = config["basicFeedBiasArmAngleY"].as<Expression>()
                            + config["basicFusedArmAngleY"].as<Expression>() * fusedYFeed
                            + config["basicDFusedArmAngleY"].as<Expression>() * dFusedYFeed + armAngleYIFeed
                            + config["basicGyroArmAngleY"].as<Expression>() * gyroYFeed;

        // Disable the arm angle feedback if required
        if (!(config["basicEnableArmAngleX"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            armAngleXFeedback = armAngleXIFeed = 0.0;
        }
        if (!(config["basicEnableArmAngleY"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            armAngleYFeedback = armAngleYIFeed = 0.0;
        }

        // Apply the arm angle feedback
        arm.angleX += armAngleXFeedback;
        arm.angleY += armAngleYFeedback;

        // Handle saving of the current integral feedback values as halt pose offsets
        if (m_saveIFeedToHaltPose && !m_savedArmIFeed) {
            config["haltArmAngleXBias"].as<Expression>() =
                config["haltArmAngleXBias"].as<Expression>() + armAngleXIFeed;
            config["haltArmAngleY"].as<Expression>() = config["haltArmAngleY"].as<Expression>() + armAngleYIFeed;
            NUClear::log("Saved the current integrated arm feedback offsets as modifications to the halt pose");
            m_savedArmIFeed = true;
        }

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config["basicIFusedArmAngleX"].as<Expression>() != 0.0
            && config["basicEnableArmAngleX"].as<bool>()
            && config["basicGlobalEnable"].as<bool>()) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config["basicIFusedArmAngleY"].as<Expression>() != 0.0
            && config["basicEnableArmAngleY"].as<bool>()
            && config["basicGlobalEnable"].as<bool>()) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Plotting
    //

    // Plot the arm motion components
    if (plot) {
        // clang-format off
        reactor.emit(graph(std::string("bonn/arm_swing_angle_")    + (arm.cad.isLeft ? "L" : "R"), CMD.swingAngle));
        reactor.emit(graph(std::string("bonn/arm_sag_swing_")      + (arm.cad.isLeft ? "L" : "R"), armSagSwing));
        reactor.emit(graph(std::string("bonn/arm_feed_armanglex_") + (arm.cad.isLeft ? "L" : "R"), armAngleXFeedback));
        reactor.emit(graph(std::string("bonn/arm_feed_armangley_") + (arm.cad.isLeft ? "L" : "R"), armAngleYFeedback));
        reactor.emit(graph(std::string("bonn/arm_abs_armext_")     + (arm.cad.isLeft ? "L" : "R"), arm.extension));
        reactor.emit(graph(std::string("bonn/arm_abs_armanglex_")  + (arm.cad.isLeft ? "L" : "R"), arm.angleX));
        reactor.emit(graph(std::string("bonn/arm_abs_armangley_")  + (arm.cad.isLeft ? "L" : "R"), arm.angleY));
        //clang-format on
    }
}

// Generate the inverse leg motion
// 'leg' is assumed to contain the desired abstract motion
void GaitEngine::inverseLegMotion(pose::InverseLegPose& leg) {
    //
    // Common motion data
    //

    // Calculate the common motion data
    // Note: We don't zero out the swingAngle here if tuning says we should because we use it for virtual slope walking
    // not swinging in this function
    CommonMotionData CMD = calcCommonMotionData(leg.cld.isLeft == m_leftLegFirst);

    //
    // Basic feedback mechanisms (footPosX, footPosY)
    //

    // Apply fused angle and gyro feedback
    double comShiftXFeedback = 0.0;
    double comShiftYFeedback = 0.0;
    if (!config["tuningNoLegFeedback"].as<bool>()) {
        // Compute the CoM shifting feedback
        comShiftXFeedback = config["basicFeedBiasComShiftX"].as<Expression>() + config["basicFusedComShiftX"].as<Expression>() * fusedYFeed
                            + config["basicDFusedComShiftX"].as<Expression>() * dFusedYFeed + config["basicIFusedComShiftX"].as<Expression>() * iFusedYFeed
                            + config["basicGyroComShiftX"].as<Expression>() * gyroYFeed;
        comShiftYFeedback = -(config["basicFeedBiasComShiftY"].as<Expression>() + config["basicFusedComShiftY"].as<Expression>() * fusedXFeed
                              + config["basicDFusedComShiftY"].as<Expression>() * dFusedXFeed + config["basicIFusedComShiftY"].as<Expression>() * iFusedXFeed
                              + config["basicGyroComShiftY"].as<Expression>() * gyroXFeed);

        // Apply the required limits if enabled
        if (config["basicComShiftXUseLimits"].as<bool>()) {
            comShiftXFeedback = utility::math::clampSoft(double(config["basicComShiftXMin"].as<Expression>()),
                                                         comShiftXFeedback,
                                                         double(config["basicComShiftXMax"].as<Expression>()),
                                                         double(config["basicComShiftXBuf"].as<Expression>()));
        }
        if (config["basicComShiftYUseLimits"].as<bool>()) {
            comShiftYFeedback = utility::math::clampSoft(double(config["basicComShiftYMin"].as<Expression>()),
                                                         comShiftYFeedback,
                                                         double(config["basicComShiftYMax"].as<Expression>()),
                                                         double(config["basicComShiftYBuf"].as<Expression>()));
        }

        // Disable the CoM shifting feedback if required
        if (!(config["basicEnableComShiftX"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            comShiftXFeedback = 0.0;
        }
        if (!(config["basicEnableComShiftY"].as<bool>() && config["basicGlobalEnable"].as<bool>())) {
            comShiftYFeedback = 0.0;
        }

        // Apply the CoM shifting feedback
        leg.footPos.x() += comShiftXFeedback;
        leg.footPos.y() += comShiftYFeedback;

        // Work out whether iFusedX contributed anything to the CPG gait
        if (haveIFusedXFeed && config["basicIFusedComShiftY"].as<Expression>() != 0.0 && config["basicComShiftYMin"].as<Expression>() < 0.0
            && config["basicComShiftYMax"].as<Expression>() > 0.0 && config["basicEnableComShiftY"].as<bool>() && config["basicGlobalEnable"].as<bool>()) {
            iFusedXLastTime = in.timestamp;
            usedIFusedX     = true;
        }

        // Work out whether iFusedY contributed anything to the CPG gait
        if (haveIFusedYFeed && config["basicIFusedComShiftX"].as<Expression>() != 0.0 && config["basicComShiftXMin"].as<Expression>() < 0.0
            && config["basicComShiftXMax"].as<Expression>() > 0.0 && config["basicEnableComShiftX"].as<bool>() && config["basicGlobalEnable"].as<bool>()) {
            iFusedYLastTime = in.timestamp;
            usedIFusedY     = true;
        }
    }

    //
    // Virtual slope leg lifting (footPosZ)
    //

    // Adjust the lift height of the foot depending on the virtual slope (a slope derived from the pitch fused angle and
    // a configured offset). This has the qualitative effect that the robot lifts its feet more when it is falling
    // forwards.
    double virtualComponent = 0.0;
    if (config["basicEnableVirtualSlope"].as<bool>() && config["basicGlobalEnable"].as<bool>() && !config["tuningNoLegVirtual"].as<bool>()) {
        double endVirtualSlope = virtualSlope * CMD.gcvX;
        if (endVirtualSlope >= 0.0) {
            virtualComponent = 0.5 * (CMD.swingAngle + 1.0) * endVirtualSlope;
        }
        else {
            virtualComponent = 0.5 * (CMD.swingAngle - 1.0) * endVirtualSlope;
        }
        leg.footPos.z() += virtualComponent;
    }

    //
    // Plotting
    //

    // Plot the inverse leg motion components
    if (plot) {
        //clang-format off
        reactor.emit(graph(std::string("bonn/leg_feed_comshiftx_") + (leg.cld.isLeft ? "L" : "R"), comShiftXFeedback));
        reactor.emit(graph(std::string("bonn/leg_feed_comshifty_") + (leg.cld.isLeft ? "L" : "R"), comShiftYFeedback));
        reactor.emit(graph(std::string("bonn/leg_virtual_slope_")  + (leg.cld.isLeft ? "L" : "R"), virtualSlope));
        reactor.emit(graph(std::string("bonn/leg_virtual_comp_")   + (leg.cld.isLeft ? "L" : "R"), virtualComponent));
        //clang-format on
    }
}

// Abstract pose coercion function
void GaitEngine::clampAbstractPose(pose::AbstractPose& pose) {
    // Coerce each of the limbs in the abstract pose
    clampAbstractArmPose(pose.leftArm);
    clampAbstractArmPose(pose.rightArm);
    clampAbstractLegPose(pose.leftLeg);
    clampAbstractLegPose(pose.rightLeg);
}

// Abstract arm pose coercion function
void GaitEngine::clampAbstractArmPose(pose::AbstractArmPose& arm) {
    // Apply the required limits if enabled
    if (config["limArmAngleXUseLimits"].as<bool>()) {
        arm.angleX = arm.cad.limbSign
                     // Minimum is negative towards inside, maximum is positive towards outside
                     * utility::math::clampSoft(double(config["limArmAngleXMin"].as<Expression>()),
                                                arm.angleX / arm.cad.limbSign,
                                                double(config["limArmAngleXMax"].as<Expression>()),
                                                double(config["limArmAngleXBuf"].as<Expression>()));
    }
    if (config["limArmAngleYUseLimits"].as<bool>()) {
        arm.angleY = utility::math::clampSoft(
            double(config["limArmAngleYMin"].as<Expression>()), arm.angleY, double(config["limArmAngleYMax"].as<Expression>()), double(config["limArmAngleYBuf"].as<Expression>()));
    }
}

// Abstract leg pose coercion function
void GaitEngine::clampAbstractLegPose(pose::AbstractLegPose& leg) {
    // Apply the required limits if enabled
    if (config["limLegAngleXUseLimits"].as<bool>())
        // Minimum is negative towards inside, maximum is positive towards outside
        leg.angleX =
            leg.cld.limbSign
            * utility::math::clampSoft(
                  double(config["limLegAngleXMin"].as<Expression>()), leg.angleX / leg.cld.limbSign, double(config["limLegAngleXMax"].as<Expression>()), 4.0);
    if (config["limLegAngleYUseLimits"].as<bool>()) {
        leg.angleY =
            utility::math::clampSoft(double(config["limLegAngleYMin"].as<Expression>()), leg.angleY, double(config["limLegAngleYMax"].as<Expression>()), 4.0);
    }
    if (config["limFootAngleXUseLimits"].as<bool>()) {
        // Minimum is negative towards inside, maximum is positive towards outside
        leg.footAngleX = leg.cld.limbSign
                         * utility::math::clampSoft(double(config["limFootAngleXMin"].as<Expression>()),
                                                    leg.footAngleX / leg.cld.limbSign,
                                                    double(config["limFootAngleXMax"].as<Expression>()),
                                                    double(config["limFootAngleXBuf"].as<Expression>()));
    }
    if (config["limFootAngleYUseLimits"].as<bool>()) {
        leg.footAngleY = utility::math::clampSoft(double(config["limFootAngleYMin"].as<Expression>()),
                                                  leg.footAngleY,
                                                  double(config["limFootAngleYMax"].as<Expression>()),
                                                  double(config["limFootAngleYBuf"].as<Expression>()));
    }
    if (config["limLegExtUseLimits"].as<bool>()) {
        leg.extension =
            utility::math::clampSoftMin(double(config["limLegExtMin"].as<Expression>()), leg.extension, double(config["limLegExtBuf"].as<Expression>()));
    }
}

// Update outputs function
void GaitEngine::updateOutputs() {
    // Transcribe the joint commands
    out.jointCmd        = m_jointPose.writeJointPosArray();
    out.jointEffort     = m_jointPose.writeJointEffortArray();
    out.useRawJointCmds = haltUseRawJointCmds;

    // Transcribe the walking flag
    out.walking = m_walking || m_blending;

    // Transcribe the leg support coefficients
    out.supportCoeffLeftLeg  = m_jointPose.leftLeg.cld.supportCoeff;
    out.supportCoeffRightLeg = m_jointPose.rightLeg.cld.supportCoeff;

    // Update the odometry
    updateOdometry();
}

// Reset the blending variables
void GaitEngine::resetBlending(double b) {
    // Disable the blending variables
    m_blending      = false;
    m_b_current     = b;
    m_b_initial     = b;
    m_b_target      = b;
    m_blendPhase    = 0.0;
    m_blendEndPhase = 0.0;
}

// Set a new blend target (USE_CALC_POSE = 0, USE_HALT_POSE = 1, and everything inbetween is interpolation)
void GaitEngine::setBlendTarget(double target,
                                double phaseTime)  // phaseTime is the gait phase in which to complete the blend
{
    // Blend target range checking
    target = utility::math::clamp(0.0, target, 1.0);

    // Immediately change the current blending factor if the required phase time is non-positive, or the current blend
    // factor is already equal to the target blend factor
    if (phaseTime <= 0.0 || m_b_current == target) {
        resetBlending(target);
        return;
    }

    // Update the blending variables
    m_blending      = true;
    m_b_initial     = m_b_current;
    m_b_target      = target;
    m_blendPhase    = 0.0;
    m_blendEndPhase = phaseTime;
}

// Evaluate the current blend factor
double GaitEngine::blendFactor() {
    // Update the current blend factor if we are in the process of blending
    if (m_blending) {
        // Should never happen...
        if (m_blendPhase < 0.0) {
            m_blendPhase = 0.0;
            m_b_current  = m_b_initial;
        }
        // Done with our blend...
        else if (m_blendPhase >= m_blendEndPhase) {
            resetBlending(m_b_target);  // Note: This internally sets m_blending to false
        }
        // In the process of blending...
        else {
            double u    = std::sin(M_PI_2 * m_blendPhase / m_blendEndPhase);
            m_b_current = m_b_initial + (u * u) * (m_b_target - m_b_initial);
        }
    }

    // Return the current blend factor
    return m_b_current;
}

// Reset the motion stance adjustment variables
void GaitEngine::resetMotionStance() {
    // Reset variables
    m_motionLegAngleXFact = 1.0;  // Feet normal
}

}  // namespace gait
