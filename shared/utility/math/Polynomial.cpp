#include "Polynomial.h"

namespace utility {
namespace math {

    const std::vector<double>& Polynomial::getCoeffs() const {
        return coeffs;
    }

    std::vector<double>& Polynomial::getCoeffs() {
        return coeffs;
    }

    const double& Polynomial::operator()(size_t index) const {
        return coeffs.at(index);
    }

    double& Polynomial::operator()(size_t index) {
        return coeffs.at(index);
    }

    size_t Polynomial::degree() const {
        return coeffs.size() - 1;
    }

    double Polynomial::pos(double x) const {
        double xx  = 1.0;
        double val = 0.0;

        for (size_t i = 0; i < coeffs.size(); i++) {
            val += xx * coeffs[i];
            xx *= x;
        }

        return val;
    }

    double Polynomial::vel(double x) const {
        double xx  = 1.0;
        double val = 0.0;

        for (size_t i = 1; i < coeffs.size(); i++) {
            val += i * xx * coeffs[i];
            xx *= x;
        }

        return val;
    }

    double Polynomial::acc(double x) const {
        double xx  = 1.0;
        double val = 0.0;

        for (size_t i = 2; i < coeffs.size(); i++) {
            val += (i - 1) * i * xx * coeffs[i];
            xx *= x;
        }

        return val;
    }

    void Polynomial::operator*=(double coef) {
        for (size_t i = 0; i < coeffs.size(); i++) {
            coeffs[i] *= coef;
        }
    }

    void Polynomial::operator+=(const Polynomial& p) {
        while (p.coeffs.size() > coeffs.size()) {
            coeffs.push_back(0.0);
        }

        for (size_t i = 0; i < p.coeffs.size(); i++) {
            coeffs[i] += p.coeffs[i];
        }
    }

}  // namespace math
}  // namespace utility
