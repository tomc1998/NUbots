#ifndef UTILITY_MATH_GEOMETRY_SPLINE_H
#define UTILITY_MATH_GEOMETRY_SPLINE_H

#include <iostream>
#include <vector>

#include "utility/math/Polynomial.h"

namespace utility {
namespace math {
    namespace geometry {

        /**
         * Spline
         *
         * Generic one dimensional polynomial spline generator
         */
        class Spline {
        public:
            /**
             * Return spline interpolation at given t. Compute spline value, its first and second derivative
             */
            double pos(double t) const;
            double vel(double t) const;
            double acc(double t) const;

            /**
             * Return spline interpolation value, first and second derivative with given t bound between 0 and 1
             */
            double posMod(double t) const;
            double velMod(double t) const;
            double accMod(double t) const;

            /**
             * Return minimum and maximum abscissa value for which spline is defined
             */
            double min() const;
            double max() const;

            /**
             * Write and read splines data into given iostream in ascii format
             */
            void exportData(std::ostream& os) const;
            void importData(std::istream& is);

        protected:
            /**
             * Internal spline part structure with a polynomial valid on an interval
             */
            struct Spline_t {
                utility::math::Polynomial poly;
                double min;
                double max;
            };

            /**
             * Spline part container
             */
            std::vector<Spline_t> splines;

            /**
             * Return spline interpolation of given value and used given polynomial evaluation function
             * (member function pointer)
             */
            double interpolation(double x, double (utility::math::Polynomial::*func)(double) const) const;

            /**
             * Return interpolation with x bound between 0 and 1
             */
            double interpolationMod(double x, double (utility::math::Polynomial::*func)(double) const) const;
        };

    }  // namespace geometry
}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_GEOMETRY_CUBICSPLINE_H
