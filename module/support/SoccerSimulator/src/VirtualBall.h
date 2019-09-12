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
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_SUPPORT_VIRTUALBALL
#define MODULE_SUPPORT_VIRTUALBALL

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

#include "message/input/Image.h"
#include "message/input/Sensors.h"
#include "message/vision/Ball.h"

#include "utility/math/matrix/Transform2D.h"

namespace module {
namespace support {

    class VirtualBall {
    public:
        VirtualBall();

        VirtualBall(const Eigen::Vector2d& position, float diameter);

        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        float diameter;

        std::mt19937 rd;
        std::normal_distribution<> angularDistribution = std::normal_distribution<>(0.0, M_PI_2);
        std::normal_distribution<> radialDistribution  = std::normal_distribution<>(0.0, 0.01);

        message::vision::Balls detect(const message::input::Image& image,
                                      const Eigen::Affine2d& robotPose,
                                      const message::input::Sensors& sensors,
                                      const Eigen::Vector4d& error);
    };
}  // namespace support
}  // namespace module

#endif  // MODULE_SUPPORT_VIRTUALBALL
