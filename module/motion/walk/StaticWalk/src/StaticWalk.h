
#ifndef MODULE_MOTION_WALK_STATICWALK_H
#define MODULE_MOTION_WALK_STATICWALK_H

#include <Eigen/Geometry>
#include <nuclear>

namespace module {
namespace motion {
    namespace walk {

        class StaticWalk : public NUClear::Reactor {

        private:
            // Transform from support foot to swing foot
            // Eigen::Affine3d Hwg;
            Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Hwg;
            // Ground space method
            Eigen::Affine3d getGroundSpace(Eigen::Affine3d& Hts, Eigen::Affine3d& Htworld);
            // The states that the robots enters to complete the steps
            enum State { INITIAL, LEFT_LEAN, RIGHT_STEP, RIGHT_LEAN, LEFT_STEP } state;
            // The time each phase takes to complete
            NUClear::clock::duration phase_time;
            // The time at the start of the current phase
            NUClear::clock::time_point start_phase;
            // The height of the robots torso (m)
            double torso_height;
            // The width of the robots stance as it walks (m)
            double stance_width;
            bool start_right_lean;
            double y_offset;
            double x_offset;
            double time;
            double rotation_limit;
            size_t subsumptionId;

        public:
            /// @brief Called by the powerplant to build and setup the StaticWalk reactor.
            explicit StaticWalk(std::unique_ptr<NUClear::Environment> environment);
        };

    }  // namespace walk
}  // namespace motion
}  // namespace module

#endif  // MODULE_MOTION_WALK_STATICWALK_H
