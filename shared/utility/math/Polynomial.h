#ifndef UTILITY_MATH_POLYNOMIAL_H
#define UTILITY_MATH_POLYNOMIAL_H
#include <cstdlib>
#include <vector>

namespace utility {
namespace math {
    /**
     * Polynomial
     *
     * Simple one dimensional
     * polynomial class for spline
     * generation
     */
    class Polynomial {
    public:
        /**
         * Default and inital degree initialization
         */
        Polynomial() : coeffs() {}
        Polynomial(unsigned int degree) : coeffs(degree + 1, 0.0) {}

        /**
         * Access to coefficient
         * indexed from constant to
         * higher degree
         */
        const std::vector<double>& getCoeffs() const;
        std::vector<double>& getCoeffs();

        /**
         * Access to coefficient
         */
        const double& operator()(size_t index) const;
        double& operator()(size_t index);

        /**
         * Return Polynomial degree
         * -1 mean empty Polynomial
         */
        size_t degree() const;

        /**
         * Polynomial evaluation, its first and
         * second derivative at given x
         */
        double pos(double x) const;
        double vel(double x) const;
        double acc(double x) const;

        /**
         * Some useful operators
         */
        void operator*=(double coef);
        void operator+=(const Polynomial& p);

    private:
        /**
         * Polynomial coefficients
         */
        std::vector<double> coeffs;
    };

}  // namespace math
}  // namespace utility

#endif  // UTILITY_MATH_POLYNOMIAL_H
