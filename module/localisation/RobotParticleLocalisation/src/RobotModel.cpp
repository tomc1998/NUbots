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

#include "RobotModel.h"

#include <armadillo>
#include <iostream>
#include <nuclear>

#include "message/input/Sensors.h"
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


    using utility::math::matrix::Transform3D;
    arma::vec::fixed<RobotModel::size> RobotModel::timeUpdate(const arma::vec::fixed<RobotModel::size>& state,
                                                              double /*deltaT*/) {
        arma::vec::fixed<RobotModel::size> new_state = state;

        return new_state;
    }


    /// Return the predicted observation of an object at the given position
    arma::vec RobotModel::predictedObservation(const arma::vec::fixed<RobotModel::size>& state,
                                               const arma::vec& actual_position,
                                               const utility::math::matrix::Transform3D& Hcw,
                                               const Goal::MeasurementType& type,
                                               const FieldDescription& fd) {

        Transform3D Hfw = fieldStateToTransform3D(state);
        Transform3D Hcf = Hcw * Hfw.i();

        // rZFf = vector from field origin to zenith high in the sky
        arma::vec3 rZFf = {0, 0, 1};
        arma::vec3 rZCc = Hcf.transformVector(rZFf);

        if (type == Goal::MeasurementType::CENTRE) {
            // rGCc = vector from camera to goal post expected position
            arma::vec3 rGCc     = Hcf.transformPoint(actual_position);
            arma::vec3 rGCc_sph = cartesianToSpherical(rGCc);  // in r,theta,phi
            return rGCc_sph;
        }

        switch (FieldDescription::GoalpostType::Value(fd.dimensions.goalpost_type)) {
            case FieldDescription::GoalpostType::CIRCLE: {
                if (type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL) {
                    arma::vec3 rNCc   = getCylindricalPostCamSpaceNormal(type, actual_position, Hcf, fd);
                    arma::vec2 angles = {std::atan2(rNCc[1], rNCc[0]),
                                         std::atan2(rNCc[2], std::sqrt(rNCc[0] * rNCc[0] + rNCc[1] * rNCc[1]))};
                    return angles;
                }
                break;
            }
            case FieldDescription::GoalpostType::RECTANGLE: {
                if (type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL) {
                    arma::vec3 rNCc   = getSquarePostCamSpaceNormal(type, actual_position, Hcf, fd);
                    arma::vec2 angles = {std::atan2(rNCc[1], rNCc[0]),
                                         std::atan2(rNCc[2], std::sqrt(rNCc[0] * rNCc[0] + rNCc[1] * rNCc[1]))};
                    return angles;
                }
                break;
            }
        }
        return arma::vec2({0, 0});
    }


    arma::vec RobotModel::observationDifference(const arma::vec& a, const arma::vec& b) {
        return a - b;
    }

    arma::vec RobotModel::observationDifference(const arma::mat& field, const arma::mat& pts) {
        // Calculate all of the cross products
        arma::Mat<double>::fixed<16, 3> crosses;
        for (int row = 0; row < field.n_rows; ++row) {
            crosses.row(row) = arma::normalise(arma::cross(field.submat(row, 0, row, 2), field.submat(row, 4, row, 6)));
        }

        // Figure out the error for each point
        arma::vec errors(pts.n_rows);
        errors.fill(arma::datum::inf);
        for (int pt = 0; pt < pts.n_rows; ++pt) {
            arma::rowvec pt_norm = arma::normalise(pts.submat(pt, 0, pt, 2));
            for (int row = 0; row < field.n_rows; ++row) {
                arma::rowvec cross_norm = crosses.row(row);
                arma::rowvec v0         = arma::normalise(field.submat(row, 0, row, 2));
                arma::rowvec v1         = arma::normalise(field.submat(row, 4, row, 6));

                double error            = arma::dot(cross_norm, pt_norm);
                arma::rowvec projection = arma::normalise(pt_norm - cross_norm * error);
                double range            = arma::dot(v0, v1);
                double offset           = arma::dot(v0, pt_norm);

                if ((0.0 <= offset) && (offset <= range)) {
                    double angle = std::acos(error);
                    if (angle < errors(pt)) {
                        errors(pt) = angle;
                    }
                }
            }
        }

        // Find any points that didnt match a line and assign it to the closest line
        for (int pt = 0; pt < pts.n_rows; ++pt) {
            if (!std::isfinite(errors(pt))) {
                arma::vec pt_norm = arma::normalise(pts.row(pt));
                for (int row = 0; row < pts.n_rows; ++row) {
                    arma::rowvec cross_norm = crosses.row(row);
                    double angle            = std::acos(arma::dot(cross_norm, pt_norm));
                    if (angle < errors(pt)) {
                        errors(pt) = angle;
                    }
                }
            }
        }

        return errors;
    }

    /// Return the predicted observation of an object at the given position
    arma::Mat<double>::fixed<16, 8> RobotModel::predictedObservation(const arma::vec::fixed<RobotModel::size>& state,
                                                                     const utility::math::matrix::Transform3D& Hcw) {
        // Convert field lines in field space to field lines in world space
        Transform3D Hfw = fieldStateToTransform3D(state);
        Transform3D Hcf = Hcw * Hfw.i();
        arma::Mat<double>::fixed<16, 8> world_lines;
        // (AB)^T = B^T A^T
        world_lines.submat(0, 0, field_lines.n_rows - 1, 3) =
            field_lines.submat(0, 0, field_lines.n_rows - 1, 3) * Hcf.t();
        world_lines.submat(0, 4, field_lines.n_rows - 1, 7) =
            field_lines.submat(0, 4, field_lines.n_rows - 1, 7) * Hcf.t();

        return world_lines;
    }

    arma::vec::fixed<RobotModel::size> RobotModel::limitState(const arma::vec::fixed<RobotModel::size>& state) {
        auto state2 = state;
        // state2[kAngle] = normalizeAngle(state2[kAngle]);
        // TODO: Clip robot's state to the field?
        return state2;
    }

    arma::mat::fixed<RobotModel::size, RobotModel::size> RobotModel::processNoise() {
        return arma::diagmat(processNoiseDiagonal);
    }

    arma::vec3 RobotModel::getCylindricalPostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                            const arma::vec3& post_centre,
                                                            const Transform3D& Hcf,
                                                            const FieldDescription& fd) {
        if (!(type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL))
            return arma::vec(0, 0, 0);
        // rZFf = field vertical
        arma::vec3 rZFf({0, 0, 1});
        arma::vec3 rZCc = Hcf.transformVector(rZFf);
        // The vector direction across the field perpendicular to the camera view vector
        arma::vec3 rLRf = arma::normalise(arma::cross(rZCc, arma::vec3({1, 0, 0})));

        float dir          = (type == Goal::MeasurementType::LEFT_NORMAL) ? 1 : -1;
        arma::vec3 rG_blCc = post_centre + 0.5 * dir * fd.dimensions.goalpost_width * rLRf;
        arma::vec3 rG_tlCc = rG_blCc + fd.dimensions.goal_crossbar_height * rZCc;

        // creating the normal vector (following convention stipulated in VisionObjects)
        return (type == Goal::MeasurementType::LEFT_NORMAL) ? arma::normalise(arma::cross(rG_blCc, rG_tlCc))
                                                            : arma::normalise(arma::cross(rG_tlCc, rG_blCc));
    }

    arma::vec3 RobotModel::getSquarePostCamSpaceNormal(const message::vision::Goal::MeasurementType& type,
                                                       const arma::vec3& post_centre,
                                                       const Transform3D& Hcf,
                                                       const FieldDescription& fd) {
        if (!(type == Goal::MeasurementType::LEFT_NORMAL || type == Goal::MeasurementType::RIGHT_NORMAL))
            return arma::vec(0, 0, 0);

        // Finding 4 corners of goalpost and centre (4 corners and centre)
        arma::mat goalBaseCorners(4, 5);
        goalBaseCorners.each_col() = arma::vec4({post_centre[0], post_centre[1], post_centre[2], 1});
        //
        goalBaseCorners.col(1) +=
            arma::vec4({0.5 * fd.dimensions.goalpost_depth, 0.5 * fd.dimensions.goalpost_width, 0, 0});
        goalBaseCorners.col(2) +=
            arma::vec4({0.5 * fd.dimensions.goalpost_depth, -0.5 * fd.dimensions.goalpost_width, 0, 0});
        goalBaseCorners.col(3) +=
            arma::vec4({-0.5 * fd.dimensions.goalpost_depth, 0.5 * fd.dimensions.goalpost_width, 0, 0});
        goalBaseCorners.col(4) +=
            arma::vec4({-0.5 * fd.dimensions.goalpost_depth, -0.5 * fd.dimensions.goalpost_width, 0, 0});

        arma::mat goalTopCorners = goalBaseCorners;
        goalTopCorners.each_col() += arma::vec4({0, 0, fd.dimensions.goal_crossbar_height, 0});

        // Transform to robot camera space
        arma::mat goalBaseCornersCam = Hcf * goalBaseCorners;
        arma::mat goalTopCornersCam  = Hcf * goalTopCorners;

        // Get widest line
        int widest          = 0;
        float largest_angle = 0;
        for (int i = 1; i < goalBaseCornersCam.n_cols; i++) {
            float angle = std::acos(arma::dot(goalBaseCornersCam.col(i), goalBaseCornersCam.col(0)));
            // Left side will have cross product point in neg field z direction
            float left_side = arma::dot(arma::cross(goalBaseCornersCam.col(i), goalBaseCornersCam.col(0)),
                                        goalTopCornersCam - goalBaseCornersCam)
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
                   ? arma::normalise(arma::cross(goalBaseCornersCam.col(widest), goalTopCornersCam.col(widest)))
                   : arma::normalise(arma::cross(goalTopCornersCam.col(widest), goalBaseCornersCam.col(widest)));
    }
}  // namespace localisation
}  // namespace module
