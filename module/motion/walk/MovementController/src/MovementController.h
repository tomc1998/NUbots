#ifndef MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
#define MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>
#include "message/input/Sensors.h"
#include "message/motion/FootTarget.h"
#include "message/motion/TorsoTarget.h"

namespace module {
namespace motion {
    namespace walk {
        class MovementController : public NUClear::Reactor {
        private:
            Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Ht_ong;
            // Returns vector from the vector field of where the foot should move on x-axis
            double f_x(const Eigen::Vector3d& vec);
            // Returns vector from the vector field of where the foot should move on y-axis (in plane space)
            double f_z(const Eigen::Vector3d& vec);
            // Plans where to move the torso to next
            Eigen::Affine3d next_torso(const double time_left,
                                       const Eigen::Affine3d& Htg,
                                       const Eigen::Affine3d& Ht_tg);
            // Plans where to move the swing foot to next
            Eigen::Affine3d next_swing(const double time_left,
                                       const Eigen::Affine3d& Htg,
                                       const Eigen::Affine3d& Hwg,
                                       const Eigen::Affine3d& Hw_tg,
                                       bool lift);
            // Height that the robot lifts its foot to step
            double step_height;
            // How steep the foot comes down to its target
            double step_steep;
            // Size of the well near the target
            double well_width;
            // Constant for mathematical function based on step_height, step_steep and well_width
            double c;
            // Gain for the feet movements
            // Eigen::VectorXd gains;
            // How far in the future we are sending the movements
            double time_horizon;

            double gain;
            double s_gain;

            bool Htong_flag;
            double velocity_limit;
            double rotation_limit;

        public:
            /// @brief Called by the powerplant to build and setup the MovementController reactor.
            explicit MovementController(std::unique_ptr<NUClear::Environment> environment);
        };
    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_MOVEMENTCONTROLLER_H
