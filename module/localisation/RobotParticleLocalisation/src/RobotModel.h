/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MODULES_LOCALISATION_ROBOTMODEL_H
#define MODULES_LOCALISATION_ROBOTMODEL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "message/input/Sensors.h"
#include "message/support/FieldDescription.h"
#include "message/vision/Goal.h"
#include "utility/input/ServoID.h"
#include "utility/localisation/transform.h"
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/support/eigen_armadillo.h"

namespace module {
namespace localisation {

    using message::input::Sensors;
    using message::support::FieldDescription;
    using message::vision::Goal;
    using utility::input::ServoID;
    using utility::localisation::fieldStateToTransform3D;
    using utility::math::angle::normalizeAngle;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::math::matrix::Transform3D;

    template <typename Scalar>
    class RobotModel {
    public:
        static constexpr size_t size = 3;

        using StateVec = Eigen::Matrix<Scalar, size, 1>;
        using StateMat = Eigen::Matrix<Scalar, size, size>;


        enum Components : int {
            // Field center in world space
            kX = 0,
            kY = 1,
            // Angle is the angle from the robot world forward direction to the field forward direction
            kAngle = 2
        };


        RobotModel() {}

        StateVec time(const StateVec& state, double deltaT) {
            Eigen::Matrix<Scalar, RobotModel<Scalar>::size, 1> new_state = state;
            return new_state;
        }

        Eigen::VectorXd predictedObservation(const StateVec& state,
                                             const Eigen::VectorXd& actual_position,
                                             const Eigen::Affine3d& Hcw,
                                             const message::vision::Goal::MeasurementType& type,
                                             const message::support::FieldDescription& fd) {

            double c = std::cos(state[2]);
            double s = std::sin(state[2]);
            Eigen::Matrix<Scalar, 3, 3> rotation;
            rotation << c << -s << 0 << s << c << 0 << 0 << 0 << 1;
            Eigen::Affine3d Hfw;
            Hfw.translation()   = Eigen::Vector3d{state[0], state[1], 0};
            Hfw.linear()        = rotation;
            Eigen::Affine3d Hcf = Hcw * Hfw.inverse();

            // rZFf = vector from field origin to zenith high in the sky
            Eigen::Vector4d rZFf = {0, 0, 1, 0};
            rZFf                 = Hcf * rZFf;

            if (type == Goal::MeasurementType::CENTRE) {
                // rGCc = vector from camera to goal post expected position
                Eigen::Vector4d rGCc_4   = {actual_position[0], actual_position[1], actual_position[2], 1};
                rGCc_4                   = Hcf * rGCc_4;
                Eigen::Vector3d rGCc     = {rGCc_4[0], rGCc_4[1], rGCc_4[2]};
                Eigen::Vector3d rGCc_sph = cartesianToSpherical(rGCc);  // in r,theta,phi
                return rGCc_sph;
            }

            switch (FieldDescription::GoalpostType::Value(fd.dimensions.goalpost_type)) {
                case FieldDescription::GoalpostType::CIRCLE: {
                    if (type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL) {
                        Eigen::Vector3d rNCc   = getCylindricalPostCamSpaceNormal(type, actual_position, Hcf, fd);
                        Eigen::Vector2d angles = {
                            std::atan2(rNCc[1], rNCc[0]),
                            std::atan2(rNCc[2], std::sqrt(rNCc[0] * rNCc[0] + rNCc[1] * rNCc[1]))};
                        return angles;
                    }
                    break;
                }
                case FieldDescription::GoalpostType::RECTANGLE: {
                    if (type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL) {
                        Eigen::Vector3d rNCc   = getSquarePostCamSpaceNormal(type, actual_position, Hcf, fd);
                        Eigen::Vector2d angles = {
                            std::atan2(rNCc[1], rNCc[0]),
                            std::atan2(rNCc[2], std::sqrt(rNCc[0] * rNCc[0] + rNCc[1] * rNCc[1]))};
                        return angles;
                    }
                    break;
                }
            }
            return Eigen::Vector2d({0, 0});
        }

        StateVec limit(const StateVec& state) {
            auto state2 = state;
            // state2[kAngle] = normalizeAngle(state2[kAngle]);
            // TODO: Clip robot's state to the field?
            return state2;
        }

        StateMat noise(const Scalar& deltaT) {
            return processNoiseDiagonal.asDiagonal() * deltaT;
        }

        template <typename... Args>
        Eigen::Matrix<Scalar, 1, 1> predict(const StateVec& state, const Args&... params) {

            // Our prediction is the first state
            return Eigen::Matrix<Scalar, 1, 1>(state[kX]);
        }

        template <typename T, typename U>
        static auto difference(const T& a, const U& b) {
            return a - b;
        }

        Eigen::Vector3d processNoiseDiagonal;

        // number and range of reset particles
        int n_rogues               = 0;
        Eigen::Vector3d resetRange = {0, 0, 0};

        // Getters
        int getRogueCount() const {
            return n_rogues;
        }
        Eigen::VectorXd getRogueRange() const {
            return resetRange;
        }

        Eigen::Vector3d getCylindricalPostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                         const Eigen::Vector3d& post_centre,
                                                         const Eigen::Affine3d& Hcf,
                                                         const message::support::FieldDescription& fd) {
            if (!(type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL))
                return Eigen::VectorXd{0, 0, 0};
            // rZFf = field vertical
            Eigen::Vector4d rZFf = {0, 0, 1, 0};
            rZFf                 = Hcf * rZFf;
            Eigen::Vector3d rZCc = {rZFf[0], rZFf[1], rZFf[2]};

            // The vector direction across the field perpendicular to the camera view vector
            Eigen::Vector3d rLRf = rZCc.cross(Eigen::Vector3d({1, 0, 0})).normalized();

            float dir               = (type == Goal::MeasurementType::LEFT_NORMAL) ? 1.0 : -1.0;
            Eigen::Vector3d rG_blCc = post_centre + 0.5 * dir * fd.dimensions.goalpost_width * rLRf;
            Eigen::Vector3d rG_tlCc = rG_blCc + fd.dimensions.goal_crossbar_height * rZCc;

            // creating the normal vector (following convention stipulated in VisionObjects)
            return (type == Goal::MeasurementType::LEFT_NORMAL) ? rG_blCc.cross(rG_tlCc).normalized()
                                                                : rG_tlCc.cross(rG_blCc).normalized();
        }

        Eigen::Vector3d getSquarePostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                    const Eigen::Vector3d& post_centre,
                                                    const Eigen::Affine3d& Hcf,
                                                    const message::support::FieldDescription& fd) {
            if (!(type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL))
                return Eigen::Vector3d{0, 0, 0};

            // Finding 4 corners of goalpost and centre (4 corners and centre)
            Eigen::Matrix<Scalar, 4, 5> goalBaseCorners;
            for (int i = 0; i < 5; ++i)
                goalBaseCorners.col(i) = Eigen::Vector4d({post_centre[0], post_centre[1], post_centre[2], 1});
            //
            goalBaseCorners.col(1) +=
                Eigen::Vector4d({0.5 * fd.dimensions.goalpost_depth, 0.5 * fd.dimensions.goalpost_width, 0, 0});
            goalBaseCorners.col(2) +=
                Eigen::Vector4d({0.5 * fd.dimensions.goalpost_depth, -0.5 * fd.dimensions.goalpost_width, 0, 0});
            goalBaseCorners.col(3) +=
                Eigen::Vector4d({-0.5 * fd.dimensions.goalpost_depth, 0.5 * fd.dimensions.goalpost_width, 0, 0});
            goalBaseCorners.col(4) +=
                Eigen::Vector4d({-0.5 * fd.dimensions.goalpost_depth, -0.5 * fd.dimensions.goalpost_width, 0, 0});

            Eigen::Matrix<Scalar, 4, 5> goalTopCorners = goalBaseCorners;
            for (int i = 0; i < 5; ++i)
                goalTopCorners.col(i) += Eigen::Vector4d({0, 0, fd.dimensions.goal_crossbar_height, 0});

            // Transform to robot camera space
            Eigen::Matrix<Scalar, 4, 5> goalBaseCornersCam = Hcf * goalBaseCorners;
            Eigen::Matrix<Scalar, 4, 5> goalTopCornersCam  = Hcf * goalTopCorners;

            // Get widest line
            int widest          = 0;
            float largest_angle = 0;
            for (int i = 1; i < goalBaseCornersCam.n_cols; i++) {
                float angle = std::acos(goalBaseCornersCam.col(i).dot(goalBaseCornersCam.col(0)));
                // Left side will have cross product point in neg field z direction
                bool left_side = (goalBaseCornersCam.col(i)
                                      .cross(goalBaseCornersCam.col(0))
                                      .dot(goalTopCornersCam - goalBaseCornersCam))
                                 < 0;
                if (left_side && (type == Goal::MeasurementType::LEFT_NORMAL)) {
                    if (angle > largest_angle) {
                        widest        = i;
                        largest_angle = angle;
                    }
                }
                else if (!left_side && (type == Goal::MeasurementType::RIGHT_NORMAL)) {
                    if (angle > largest_angle) {
                        widest        = i;
                        largest_angle = angle;
                    }
                }
            }

            // creating the normal vector (following convention stipulated in VisionObjects)
            // Normals point into the goal centre
            return (type == Goal::MeasurementType::LEFT_NORMAL)
                       ? (goalBaseCornersCam.col(widest).cross(goalTopCornersCam.col(widest))).normalize()
                       : (goalTopCornersCam.col(widest).cross(goalBaseCornersCam.col(widest))).normalize();
        }
    };
}  // namespace localisation
}  // namespace module
#endif
