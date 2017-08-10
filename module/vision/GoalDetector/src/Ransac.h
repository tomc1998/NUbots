#pragma once

#include <vector>

#include "Point.h"
#include "RANSACLine.h"

namespace RANSAC {

/**
* Implementation of the RANSAC algorithm for finding a straight line
* amongst a noisy set of points.
*
* @return           Whether or not a line has been found
*
* @param points     The dataset to look in, vector of Points
* @param result     The chosen line
* @param cons       Pointer to the concensus set array
* @param k          Maximum number of iterations of RANSAC
* @param e          Maximum distance a point can be from a line to be in its
*                   concensus set
* @param n          The minimum number of points needed to form a concensus set
* @param            seed A seed value
**/
bool findLine(const std::vector<Point>& points,
              std::vector<bool>** cons,
              RANSACLine& result,
              unsigned int k,
              float e,
              unsigned int n,
              std::vector<bool> cons_buf[2],
              unsigned int* seed);

/**
 * As above, but includes a slope constraint to keep the slope close to 1,
 * ie. slope <= slopeConstraint and slope >= 1/slopeConstraint, a negative value
 * means there is no constraint
 **/
bool findLineConstrained(const std::vector<Point>& points,
                         std::vector<bool>** cons,
                         RANSACLine& result,
                         unsigned int k,
                         float e,
                         unsigned int n,
                         std::vector<bool> cons_buf[2],
                         unsigned int* seed,
                         float slopeConstraint);
}
