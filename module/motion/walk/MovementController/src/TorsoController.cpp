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
         * @param time_left The amount of time remaining until we should hit our target
         * @param Htg homogeneous transfrom from ground space to current torso space
         * @param Ht_tg homogeneous transform from ground space to our target torso space
         * @return Ht_ng homogeneous transfrom from ground space to the horizon torso space (next torso space)
         */
        Eigen::Affine3d TorsoController::next_torso(const double& time_horizon,
                                                    const double& time_left,
                                                    const Eigen::Affine3d& Htg,
                                                    const Eigen::Affine3d& Ht_tg) {
            // Interpolate towards target
            double factor = time_horizon / time_left;

            Eigen::Affine3d Hgn;
            Hgn.translation() = Htg.inverse().translation() * (1.0 - factor) + Ht_tg.inverse().translation() * factor;

            // Eigen::Affine3d Ht_tt(Ht_tg * Htg.inverse());
            // Eigen::AngleAxisd Rnt = Eigen::AngleAxisd().fromRotationMatrix(Ht_tt.rotation());
            // Rnt.angle()           = Rnt.angle() * factor;  // Or maybe 1-factor
            // Hgn.linear() = (Rnt * Htg.rotation()).transpose();

            Hgn.linear() = Eigen::Quaterniond(Htg.rotation())
                               .slerp(factor, Eigen::Quaterniond(Ht_tg.rotation()))
                               .toRotationMatrix()
                               .transpose();

            // NUClear::log("\nTORSO CONTROLLER LOG:\n",
            //              "time_horizon:",
            //              time_horizon,
            //              "\ntime_left",
            //              time_left,
            //              "\nrTGg * (1-factor):",
            //              (Htg.inverse().translation() * (1.0 - factor)).transpose(),
            //              "\nrT_tGg * factor:",
            //              (Ht_tg.inverse().translation() * factor).transpose(),
            //              "\nHgn\n",
            //              Hgn.matrix(),
            //              "\nEND TORSO CONTROLLER LOG.\n");

            return Hgn.inverse();
        }


    }  // namespace walk
}  // namespace motion
}  // namespace module
