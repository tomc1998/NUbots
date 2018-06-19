#include <iomanip>
#include <stdexcept>

#include "Spline.h"

namespace utility {
namespace math {
    namespace geometry {

        using utility::math::Polynomial;

        double Spline::pos(double t) const {
            return interpolation(t, &Polynomial::pos);
        }

        double Spline::vel(double t) const {
            return interpolation(t, &Polynomial::vel);
        }

        double Spline::acc(double t) const {
            return interpolation(t, &Polynomial::acc);
        }

        double Spline::posMod(double t) const {
            return interpolationMod(t, &Polynomial::pos);
        }

        double Spline::velMod(double t) const {
            return interpolationMod(t, &Polynomial::vel);
        }

        double Spline::accMod(double t) const {
            return interpolationMod(t, &Polynomial::acc);
        }

        double Spline::min() const {
            if (splines.empty()) {
                return 0.0;
            }
            else {
                return splines.front().min;
            }
        }

        double Spline::max() const {
            if (splines.empty()) {
                return 0.0;
            }
            else {
                return splines.back().max;
            }
        }

        void Spline::exportData(std::ostream& os) const {
            for (size_t i = 0; i < splines.size(); i++) {
                os << std::setprecision(10) << splines[i].min << " ";
                os << std::setprecision(10) << splines[i].max << " ";
                os << std::setprecision(10) << splines[i].poly.getCoeffs().size() << " ";
                for (size_t j = 0; j < splines[i].poly.getCoeffs().size(); j++) {
                    os << std::setprecision(10) << splines[i].poly.getCoeffs()[j] << " ";
                }
            }
            os << std::endl;
        }

        void Spline::importData(std::istream& is) {
            bool isFormatError;
            while (is.good()) {
                isFormatError = true;
                double min;
                double max;
                size_t size;
                Polynomial p;
                // Load spline interval and degree
                is >> min;
                if (!is.good()) {
                    break;
                }
                is >> max;
                if (!is.good()) {
                    break;
                }
                is >> size;
                // Load polynomial coefficients
                p.getCoeffs().resize(size);
                for (size_t i = 0; i < size; i++) {
                    if (!is.good()) {
                        break;
                    }
                    is >> p.getCoeffs()[i];
                }
                // Save spline part
                isFormatError = false;
                splines.push_back({p, min, max});
                // Exit on line break
                while (is.peek() == ' ') {
                    if (!is.good()) {
                        break;
                    }
                    is.ignore();
                }
                if (is.peek() == '\n') {
                    break;
                }
            }
            if (isFormatError) {
                throw std::logic_error("Spline import format invalid");
            }
        }

        double Spline::interpolation(double x, double (Polynomial::*func)(double) const) const {
            // Bound asked abscisse into spline range
            if (x <= splines.front().min) {
                x = splines.front().min;
            }
            if (x >= splines.back().max) {
                x = splines.back().max;
            }
            // Bijection spline search
            size_t indexLow = 0;
            size_t indexUp  = splines.size() - 1;
            while (indexLow != indexUp) {
                size_t index = (indexUp + indexLow) / 2;
                if (x < splines[index].min) {
                    indexUp = index - 1;
                }
                else if (x > splines[index].max) {
                    indexLow = index + 1;
                }
                else {
                    indexUp  = index;
                    indexLow = index;
                }
            }
            // Compute and return spline value
            return (splines[indexUp].poly.*func)(x - splines[indexUp].min);
        }

        double Spline::interpolationMod(double x, double (Polynomial::*func)(double) const) const {
            if (x < 0.0) {
                x = 1.0 + (x - int(x));
            }
            else if (x > 1.0) {
                x = (x - int(x));
            }
            return interpolation(x, func);
        }

    }  // namespace geometry
}  // namespace math
}  // namespace utility
