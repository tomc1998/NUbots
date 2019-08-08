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
                                                   const Eigen::Affine3d& Hw_tg) {
            // NUClear::log("\nHwg\n", Hwg.matrix(), "\n\nHw_tg\n", Hw_tg.matrix());

            double factor = time_horizon / time_left;

            //---------------------------------------------------------------
            //-------------VECTOR FIELD--------------------------------------
            //---------------------------------------------------------------

            Eigen::Affine3d Hw_tw    = Hw_tg * Hwg.inverse();
            Eigen::Vector3d rWW_tw_t = Hw_tw.translation();
            Eigen::Vector3d rNW_tw_t =
                rWW_tw_t + (Eigen::Vector3d(f_x(rWW_tw_t), 0, f_z(rWW_tw_t)).normalized() * factor * rWW_tw_t.norm());
            Eigen::Vector3d vector(Eigen::Vector3d(f_x(rWW_tw_t), 0, f_z(rWW_tw_t)).normalized() * factor
                                   * rWW_tw_t.norm());
            rNW_tw_t.y() = rWW_tw_t.y() * factor;
            // NUClear::log(vector.x(), ", ", vector.z());
            Eigen::Vector3d rNGg = Hw_tg.inverse() * rNW_tw_t;

            //---------------------------------------------------------------
            //---------------------------------------------------------------

            double current_distance_target = rWW_tw_t.norm();

            // If we are very close to the target, just go to the target directly
            if (current_distance_target < config.well_width) {
                rNGg = Hw_tg.inverse().translation();
            }

            Eigen::Affine3d Hgn;
            Hgn.linear() = Eigen::Quaterniond(Hwg.rotation())
                               .slerp(factor, Eigen::Quaterniond(Hw_tg.rotation()))
                               .toRotationMatrix()
                               .transpose();
            // Hgn.linear()      = Eigen::Matrix3d::Identity();
            Hgn.translation() = rNGg;

            return Hgn.inverse();
        }

    }  // namespace walk
}  // namespace motion
}  // namespace module
