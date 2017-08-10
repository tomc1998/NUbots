#include "Ransac.h"
#include <math.h>

#include <limits>

bool RANSAC::findLine(const std::vector<Point>& points,
                      std::vector<bool>** cons,
                      RANSACLine& result,
                      unsigned int k,
                      float e,
                      unsigned int n,
                      std::vector<bool> cons_buf[2],
                      unsigned int* seed) {
    return RANSAC::findLineConstrained(points, cons, result, k, e, n, cons_buf, seed, -1.f);
}


bool RANSAC::findLineConstrained(const std::vector<Point>& points,
                                 std::vector<bool>** cons,
                                 RANSACLine& result,
                                 unsigned int k,
                                 float e,
                                 unsigned int n,
                                 std::vector<bool> cons_buf[2],
                                 unsigned int* seed,
                                 float slopeConstraint) {
    if (points.size() < n || n < 2) {
        return false;
    }

    /* error of best line found so far */
    float minerr = std::numeric_limits<float>::max();

    std::vector<bool>*best_concensus, *this_concensus;
    best_concensus = &cons_buf[0];

    unsigned int i, j;

    /**
     * Randomly select 2 points and create a line
     */
    for (i = 0; i < k; ++i) {
        unsigned int p1, p2;
        p1 = rand_r(seed) % points.size();
        do {
            p2 = rand_r(seed) % points.size();
        } while (p1 == p2);

        RANSACLine l(points[p1], points[p2]);
        float slope = -((float) l.t1) / ((float) l.t2);
        if (slopeConstraint > 0.f && (slope < (1 / slopeConstraint) || slope > slopeConstraint)) continue;

        /**
         * figure out the variance (sum of distances of points from the line)
         * could use dist() here, but since the denominator is consistent, we
         * save time and implement it again here.
         */

        const float denom = sqrt(l.t1 * l.t1 + l.t2 * l.t2);
        const float newe  = e * denom;

        /**
         * Choose the currently unused concensus buffer
         */

        if (best_concensus == &cons_buf[0]) {
            this_concensus = &cons_buf[1];
        }
        else {
            this_concensus = &cons_buf[0];
        }

        unsigned int n_concensus_points = 0;
        for (j = 0; j != points.size(); ++j) {
            const Point& p = points[j];
            float dist     = (l.t1 * p.x() + l.t2 * p.y() + l.t3);

            if (dist < 0) {
                dist = -dist;
            }

            if (dist < newe) {
                l.var += dist;
                ++n_concensus_points;
                (*this_concensus)[j] = true;
            }
            else {
                (*this_concensus)[j] = false;
            }
        }
        l.var /= denom;

        const float k = 0.2;
        l.var         = (k * l.var) - n_concensus_points;
        if (l.var < minerr && n_concensus_points >= n) {
            minerr = l.var;
            l.var  = l.var / (points.size() * e);
            result = l;
            // std::cout << result.t1 << " " << result.t2 << " " << result.t3 << " " << result.var << std::endl;
            best_concensus = this_concensus;
        }
    }

    if (minerr < std::numeric_limits<float>::max()) {
        *cons = best_concensus;
        return true;
    }
    else {
        return false;
    }
}
