// Gait utility class that provides abstract pose representation functionality
// File: gait_abstract_pose.cpp
// Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

// Includes
#include <cmath>

#include "utility/math/comparison.h"

#include "AbstractPose.h"
#include "InversePose.h"

// Gait namespace
namespace gait {
namespace pose {

    //
    // AbstractLegPose
    //

    // Set the abstract leg pose to a given joint leg pose
    void AbstractLegPose::setFromJointPose(const JointLegPose& pose) {
        cld = pose.cld;
        fromJointAngles(pose.hipYaw, pose.hipRoll, pose.hipPitch, pose.kneePitch, pose.anklePitch, pose.ankleRoll);
    }

    // Set the abstract leg pose to a given inverse leg pose
    void AbstractLegPose::setFromInversePose(const InverseLegPose& pose) {
        cld = pose.cld;
        double hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll;
        pose.getJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
        fromJointAngles(hipYaw, hipRoll, hipPitch, kneePitch, anklePitch, ankleRoll);
    }

    // Set the pose of the leg to the pose defined by the given joint angles
    void AbstractLegPose::fromJointAngles(double hipYaw,
                                          double hipRoll,
                                          double hipPitch,
                                          double kneePitch,
                                          double anklePitch,
                                          double ankleRoll) {
        double alpha = -0.5 * kneePitch;
        angleX       = hipRoll;
        angleY       = hipPitch - alpha;
        angleZ       = hipYaw;
        footAngleX   = ankleRoll + angleX;
        footAngleY   = anklePitch + angleY - alpha;
        extension    = 1.0 - cos(alpha);
    }

    // Calculate the joint angles corresponding to the current abstract leg pose
    void AbstractLegPose::getJointAngles(double& hipYaw,
                                         double& hipRoll,
                                         double& hipPitch,
                                         double& kneePitch,
                                         double& anklePitch,
                                         double& ankleRoll) const {
        // Note: Choosing negative alpha makes the limb bend out forwards (knee-like)
        double alpha = -std::acos(utility::math::clamp(0.0, 1.0 - extension, 1.0));
        hipYaw       = angleZ;
        hipRoll      = angleX;
        hipPitch     = angleY + alpha;
        kneePitch    = -2.0 * alpha;
        anklePitch   = footAngleY - angleY + alpha;
        ankleRoll    = footAngleX - angleX;
    }

    //
    // AbstractArmPose
    //

    // Set the abstract arm pose to a given joint arm pose
    void AbstractArmPose::setFromJointPose(const JointArmPose& pose) {
        cad = pose.cad;
        fromJointAngles(pose.shoulderPitch, pose.shoulderRoll, pose.elbowPitch);
    }

    // Set the abstract arm pose to a given inverse arm pose
    void AbstractArmPose::setFromInversePose(const InverseArmPose& pose) {
        cad = pose.cad;
        double shoulderPitch, shoulderRoll, elbowPitch;
        pose.getJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
        fromJointAngles(shoulderPitch, shoulderRoll, elbowPitch);
    }

    // Set the pose of the arm to the pose defined by the given joint angles
    void AbstractArmPose::fromJointAngles(double shoulderPitch, double shoulderRoll, double elbowPitch) {
        double alpha = -0.5 * elbowPitch;
        angleX       = shoulderRoll;
        angleY       = shoulderPitch - alpha;
        extension    = 1.0 - cos(alpha);
    }

    // Calculate the joint angles corresponding to the current abstract arm pose
    void AbstractArmPose::getJointAngles(double& shoulderPitch, double& shoulderRoll, double& elbowPitch) const {
        // Note: Choosing positive alpha makes the limb bend out backwards (elbow-like)
        double alpha  = std::acos(utility::math::clamp(0.0, 1.0 - extension, 1.0));
        shoulderPitch = angleY + alpha;
        shoulderRoll  = angleX;
        elbowPitch    = -2.0 * alpha;
    }

    //
    // AbstractPose
    //

    // Set the abstract pose to a given joint pose
    void AbstractPose::setFromJointPose(const JointPose& pose) {
        leftLeg.setFromJointPose(pose.leftLeg);
        rightLeg.setFromJointPose(pose.rightLeg);
        leftArm.setFromJointPose(pose.leftArm);
        rightArm.setFromJointPose(pose.rightArm);
    }

    // Set the abstract pose to a given inverse pose
    void AbstractPose::setFromInversePose(const InversePose& pose) {
        leftLeg.setFromInversePose(pose.leftLeg);
        rightLeg.setFromInversePose(pose.rightLeg);
        leftArm.setFromInversePose(pose.leftArm);
        rightArm.setFromInversePose(pose.rightArm);
    }

}  // namespace pose
}  // namespace gait
