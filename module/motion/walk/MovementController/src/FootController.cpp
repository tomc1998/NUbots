#include "FootController.h"

namespace module {
namespace motion {
    namespace walk {

        double FootController::f_x(const Eigen::Vector3d& pos) {
            return (-pos.x() / std::abs(pos.x()))
                   * std::exp(-std::abs(std::pow(config.c * pos.x(), -config.step_steep)));
        }

        double FootController::f_z(const Eigen::Vector3d& pos) {
            return std::exp(-std::abs(std::pow(config.c * pos.x(), -config.step_steep)))
                   - (pos.z() / config.step_height);
        }

        /**
         * Calculates the next swing foot position we should be at in time_horizon amount of time
         *
         * @param time_left The amount of time remaining until we should hit our target
         * @param Hf_wg homogeneous transform from ground space to current s"wing" (w) foot space
         * @param Hf_wtg homogeneous transform from ground space to target s"wing" (w) foot space
         *
         * @return Hf_wng homogeneous transform from ground space to the horizon swing space (next swing space)
         */
        Eigen::Affine3d FootController::next_swing(const double& time_horizon,
                                                   const double& time_left,
                                                   const Eigen::Affine3d& Hwg,
                                                   const Eigen::Affine3d& Hw_tg,
                                                   bool lift) {
            // Hw_ng is the transformation matrix we will return
            Eigen::Affine3d Hw_ng;

            // // If the target is directly in front of the foot, just do vector field with no interpolation


            Eigen::Vector3d rWGg   = Hwg.inverse().translation();
            Eigen::Vector3d rW_tGg = Hw_tg.inverse().translation();

            double factor = time_horizon / time_left;

            //---------------------------------------------------------------
            //-------------VECTOR FIELD STUFF--------------------------------
            //---------------------------------------------------------------

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


            //---------------------------------------------------------------
            //-------------VECTOR FIELD STUFF--------------------------------
            //---------------------------------------------------------------

            Eigen::Affine3d Hw_tw    = Hw_tg * Hwg.inverse();
            Eigen::Vector3d rWW_tw_t = Hw_tw.translation();
            Eigen::Vector3d rNW_tw_t =
                rWW_tw_t + (Eigen::Vector3d(f_x(rWW_tw_t), 0, f_z(rWW_tw_t)).normalized() * factor * rWW_tw_t.norm());
            rNGg = Hw_tg.inverse() * rNW_tw_t;

            //---------------------------------------------------------------
            //------------CLANK----------------------------------------------
            //---------------------------------------------------------------

            // bool ridiculous_rotation =
            //     (Eigen::Matrix3d::Identity() - Hwg.rotation() * Hw_ong.rotation().inverse()).norm() > rotation_limit;

            // if (Hwong_flag && !ridiculous_rotation) {
            //     // Get a rotation change for clanking
            //     double new_clank_rotation =
            //         (Eigen::Matrix3d::Identity() - Hw_ng.rotation() * Hw_tg.rotation().transpose()).norm();
            //     double old_clank_rotation =
            //         (Eigen::Matrix3d::Identity() - Hw_ong.rotation() * Hw_tg.rotation().transpose()).norm();
            //     if (old_clank_rotation < new_clank_rotation) {
            //         // log("ROTATION CLANK");
            //         Hw_ng.linear() = Hw_ong.linear();
            //     }
            //     else {
            //         // log("ROTATION SUCCESS!");
            //     }
            // }
            // else {
            //     // log("RIDICULOUS ROTATION (or not flag");
            // }

            // bool ridiculous = (Hwg.inverse().translation() - Hw_ong.inverse().translation()).norm() > velocity_limit;

            // if (Hwong_flag && !ridiculous) {
            //     double old_distance = (Hw_ong.inverse().translation() - Hw_tg.inverse().translation()).norm();
            //     double new_distance = (Hw_ng.inverse().translation() - Hw_tg.inverse().translation()).norm();

            //     // log("old_distance", old_distance);
            //     // log("new_distance", new_distance);

            //     if (old_distance < new_distance) {
            //         // log("TRANSLATION CLANK", "\nold_distance", old_distance, "\nnew_distance", new_distance);
            //         Hw_ng.translation() = Hw_ong.translation();
            //     }
            //     else {
            //         // log("TRANSLATION SUCCESS!");
            //     }
            // }
            // else {
            //     // log("RIDICULOUS TRANSLATION (or not flag) ");
            // }

            Hw_ong = Hw_ng;


            //---------------------------------------------------------------
            //---------------------------------------------------------------


            double current_distance_target = rWPp.norm();
            // log("distance", current_distance_target);

            // If we are very close to the target, just go to the target directly
            if (current_distance_target < config.well_width || !lift) {
                // log("lift:", lift, "distance to target:", current_distance_target);
                rNGg = rW_tGg;
            }

            // Set the translation of the matrix to the correct vector
            // Hw_ng.linear() =
            //     Eigen::Quaterniond(Hwg.linear()).slerp(factor,
            //     Eigen::Quaterniond(Hw_tg.linear())).toRotationMatrix();


            Eigen::Affine3d Hgn;
            Hgn.linear()      = Hw_tg.linear().transpose();
            Hgn.translation() = rNGg;
            Hw_ng             = Hgn.inverse();

            // Hw_ng.linear() =
            //     Eigen::Quaterniond(Hwg.linear()).slerp(factor,
            //     Eigen::Quaterniond(Hw_tg.linear())).toRotationMatrix();

            return Hw_ng;
        }

    }  // namespace walk
}  // namespace motion
}  // namespace module
