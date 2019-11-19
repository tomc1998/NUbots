#include "TorsoController.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        /**
         * Calculates the next torso postion we should be at in time_horizon amount of time
         *
         * @param time_horizon Time into the future with which we project the next position to
         * @param time_left The amount of time remaining until we should hit our target
         * @param Htg homogeneous transfrom from ground space to current torso space
         * @param Ht_tg homogeneous transform from ground space to our target torso space
         *
         * @return Hng homogeneous transfrom from ground space to the horizon torso space (next torso space)
         */
        Eigen::Affine3d TorsoController::next_torso(const double& time_horizon,
                                                    const double& time_left,
                                                    const Eigen::Affine3d& Htg,
                                                    const Eigen::Affine3d& Ht_tg) {
            // Interpolate towards target
            double factor = time_horizon / time_left;

            Eigen::Affine3d Hgn;

            // Interpolate the translation between current torso position and target position
            Hgn.translation() = Htg.inverse().translation() * (1.0 - factor) + Ht_tg.inverse().translation() * factor;

            // Slerp the rotation of the torso
            Hgn.linear() = Eigen::Quaterniond(Htg.rotation())
                               .slerp(factor, Eigen::Quaterniond(Ht_tg.rotation()))
                               .toRotationMatrix()
                               .transpose();

            return Hgn.inverse();
        }


    }  // namespace walk
}  // namespace motion
}  // namespace module
