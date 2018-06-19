#include "CubicSpline.h"

#include <algorithm>
#include <stdexcept>

namespace utility {
namespace math {
    namespace geometry {

        void CubicSpline::addPoint(double time, double position, double velocity) {
            points.push_back({time, position, velocity});
            computeSplines();
        }

        utility::math::Polynomial CubicSpline::polyFit(double t,
                                                       double pos1,
                                                       double vel1,
                                                       double pos2,
                                                       double vel2) const {
            if (t <= 0.00001) {
                throw std::logic_error("CubicSpline invalid spline interval");
            }

            double t2 = t * t;
            double t3 = t2 * t;

            utility::math::Polynomial p;
            p.getCoeffs().resize(4);
            p.getCoeffs()[0] = pos1;
            p.getCoeffs()[1] = vel1;
            p.getCoeffs()[3] = (vel2 - vel1 - 2.0 * (pos2 - pos1 - vel1 * t) / t) / t2;
            p.getCoeffs()[2] = (pos2 - pos1 - vel1 * t - p.getCoeffs()[3] * t3) / t2;

            return p;
        }

        void CubicSpline::computeSplines() {
            Spline::splines.clear();
            if (points.size() < 2) {
                return;
            }

            std::sort(points.begin(), points.end(), [](const Point& p1, const Point& p2) -> bool {
                return p1.time < p2.time;
            });

            for (size_t i = 1; i < points.size(); i++) {
                double time = points[i].time - points[i - 1].time;
                if (time > 0.00001) {
                    Spline::splines.push_back({polyFit(time,
                                                       points[i - 1].position,
                                                       points[i - 1].velocity,
                                                       points[i].position,
                                                       points[i].velocity),
                                               points[i - 1].time,
                                               points[i].time});
                }
            }
        }

    }  // namespace geometry
}  // namespace math
}  // namespace utility
