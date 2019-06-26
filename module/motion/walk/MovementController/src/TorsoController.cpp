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
                                                    const Eigen::Affine3d& Ht_tg,
                                                    const bool& ratchet) {

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

            return Hgn.inverse();

            // NUClear::log("next target translation:\n",
            //              Htg.inverse().translation().transpose(),
            //              "\n",
            //              Hgn.translation().transpose(),
            //              "\n",
            //              Ht_tg.inverse().translation().transpose(),
            //              "\n");

            //---------------------------------------------------------------
            //------RATCHET--------------------------------------------------
            //---------------------------------------------------------------

            // Calculate his position and velocity and see if he hasn't moved as he should be
            double new_rotation = std::abs(Eigen::AngleAxisd().fromRotationMatrix((Ht_tg * Hgn).rotation()).angle());
            double old_rotation = std::abs(Eigen::AngleAxisd().fromRotationMatrix((Ht_tg * Hgo).rotation()).angle());
            double old_rotation_dispacement =
                std::abs(Eigen::AngleAxisd().fromRotationMatrix((Htg * Hgo).rotation()).angle());
            double new_distance     = (Ht_tg * Hgn).translation().norm();
            double old_distance     = (Ht_tg * Hgo).translation().norm();
            double old_displacement = (Htg.inverse().translation() - Hgo.translation()).norm();

            if (ratchet && old_displacement < config.max_translation && old_distance < new_distance
                || old_rotation_dispacement < config.max_rotation && old_rotation < new_rotation) {

                // Calculate ratchet to ground
                Eigen::Affine3d Hgr = Htg.inverse();
                if (old_displacement < config.max_translation && old_distance < new_distance) {
                    Hgr.translation() = Hgo.translation();
                    NUClear::log("translation ratchet");
                }
                else if (old_displacement < config.max_translation) {
                    NUClear::log("Translation ratchet snapped");
                }
                if (old_rotation_dispacement < config.max_rotation && old_rotation < new_rotation) {
                    Hgr.linear() = Hgo.linear();
                    NUClear::log("rotation ratchet");
                }
                else if (old_rotation_dispacement < config.max_rotation) {
                    NUClear::log("Rotation ratchet snapped");
                }

                // Debugging!
                double current_rotation =
                    std::abs(Eigen::AngleAxisd().fromRotationMatrix((Ht_tg * Htg.inverse()).rotation()).angle());
                double current_distance = (Ht_tg * Htg.inverse()).translation().norm();
                NUClear::log("Ratcheting",
                             "v_o:",
                             old_displacement,
                             "v_n:",
                             config.max_translation,
                             "d_o:",
                             old_distance,
                             "d_n:",
                             new_distance,
                             "d_c:",
                             current_distance,
                             "w_o:",
                             old_rotation_dispacement,
                             "w_n:",
                             config.max_rotation,
                             "r_o:",
                             old_rotation,
                             "r_n:",
                             new_rotation,
                             "c_r:",
                             current_rotation);

                return next_torso(time_horizon, time_left, Hgr.inverse(), Ht_tg, false);
            }
            else if (ratchet) {
                NUClear::log("No Ratcheting");
            }
            else if (ratchet
                     && (old_displacement < config.max_translation || old_rotation_dispacement < config.max_rotation)) {
                NUClear::log("A ratchet snapped");
            }

            // Store our old value
            Hgo = Hgn;

            return Hgn.inverse();
        }


    }  // namespace walk
}  // namespace motion
}  // namespace module
