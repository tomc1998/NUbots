#ifndef UTILITY_MATH_GEOMETRY_CUBICSPLINE_H
#define UTILITY_MATH_GEOMETRY_CUBICSPLINE_H

#include "Spline.h"

#include "utility/math/Polynomial.h"

namespace utility {
namespace math {
    namespace geometry {

        /**
         * CubicSpline
         *
         * Implementation of 3rd order polynomial splines
         */
        class CubicSpline : public Spline {
        public:
            /**
             * Add a new point with its time, position value, and velocity
             */
            void addPoint(double time, double position, double velocity = 0.0);

        private:
            /**
             * Simple point structure
             */
            struct Point {
                double time;
                double position;
                double velocity;
            };

            /**
             * Points container
             */
            std::vector<Point> points;

            /**
             * Fit a polynomial between 0 and t with given pos, vel and acc initial and final conditions
             */
            Polynomial polyFit(double t, double pos1, double vel1, double pos2, double vel2) const;

            /**
             * Recompute splines interpolation model
             */
            void computeSplines();
        };

    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_GEOMETRY_CUBICSPLINE_H
