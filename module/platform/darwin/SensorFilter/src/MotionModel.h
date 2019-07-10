/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_PLATFORM_DARWIN_MOTIONMODEL_H
#define MODULE_PLATFORM_DARWIN_MOTIONMODEL_H

/* Motion model Motion Unit*/
#include <armadillo>

namespace module {
namespace platform {
    namespace darwin {

        class MotionModel {

        public:
            // Gravity
            static constexpr double G = -9.80665;

            // Our position in global space
            // rTWw
            static constexpr uint PX = 0;
            static constexpr uint PY = 1;
            static constexpr uint PZ = 2;

            // Our velocity in global space
            // rTWw
            static constexpr uint VX = 3;
            static constexpr uint VY = 4;
            static constexpr uint VZ = 5;

            // Our orientation from robot to world
            // Rwt
            static constexpr uint QW = 6;
            static constexpr uint QX = 7;
            static constexpr uint QY = 8;
            static constexpr uint QZ = 9;

            // Our rotational velocity in robot space
            // Gyroscope measures the angular velocity of the torso in torso space
            // omegaTTt
            static constexpr uint WX = 10;
            static constexpr uint WY = 11;
            static constexpr uint WZ = 12;

            // Gyroscope Bias
            // omegaTTt
            static constexpr uint BX = 13;
            static constexpr uint BY = 14;
            static constexpr uint BZ = 15;

            // The size of our state
            static constexpr size_t size = 16;

            // Our static process noise matrix
            arma::mat::fixed<size, size> processNoiseMatrix;

            // The velocity decay for x/y/z velocities (1.0 = no decay)
            arma::vec3 timeUpdateVelocityDecay = {1, 1, 1};

            struct MeasurementType {
                struct GYROSCOPE {};
                struct ACCELEROMETER {};
                struct FLAT_FOOT_ODOMETRY {};
                struct FLAT_FOOT_ORIENTATION {};
            };

            MotionModel() : processNoiseMatrix(arma::fill::eye) {}  // empty constructor

            arma::vec::fixed<size> timeUpdate(const arma::vec::fixed<size>& state, double deltaT);

            arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::ACCELEROMETER&);
            arma::vec3 predictedObservation(const arma::vec::fixed<size>& state, const MeasurementType::GYROSCOPE&);
            arma::vec3 predictedObservation(const arma::vec::fixed<size>& state,
                                            const MeasurementType::FLAT_FOOT_ODOMETRY&);
            arma::vec4 predictedObservation(const arma::vec::fixed<size>& state,
                                            const MeasurementType::FLAT_FOOT_ORIENTATION&);

            arma::vec observationDifference(const arma::vec& a, const arma::vec& b);

            arma::vec::fixed<size> limitState(const arma::vec::fixed<size>& state);

            const arma::mat::fixed<size, size>& processNoise();
        };
    }  // namespace darwin
}  // namespace platform
}  // namespace module

#endif  // MODULE_PLATFORM_DARWIN_MOTIONMODEL_H
