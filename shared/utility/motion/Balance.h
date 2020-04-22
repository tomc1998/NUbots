/*
 * This file is part of WalkEngine.
 *
 * WalkEngine is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * WalkEngine is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with WalkEngine.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#ifndef UTILITY_MOTION_BALANCE_H
#define UTILITY_MOTION_BALANCE_H

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nuclear>

#include "extension/Configuration.h"
#include "message/input/Sensors.h"
#include "message/motion/KinematicsModel.h"
#include "utility/input/LimbID.h"
#include "utility/math/matrix/transform.h"
//#include "utility/input/ServoID.h"

namespace utility {
namespace motion {

    class Balancer {
    private:
        // Config
        float rotationPGain = 0;
        float rotationIGain = 0;
        float rotationDGain = 0;

        float translationPGainX = 0;
        float translationPGainY = 0;
        float translationPGainZ = 0;

        float translationDGainX = 0;
        float translationDGainY = 0;
        float translationDGainZ = 0;

        float ankleRotationScale = 0;
        float hipRotationScale   = 0;

        // State
        float dPitch    = 0;
        float dRoll     = 0;
        float lastPitch = 0;
        float lastRoll  = 0;

        Eigen::Quaternion<float> lastErrorQuaternion;
        NUClear::clock::time_point lastBalanceTime;

    public:
        Balancer() : lastErrorQuaternion(), lastBalanceTime() {}
        void configure(const YAML::Node& config);
        void balance(const message::motion::KinematicsModel& hip,
                     Eigen::Affine3f footToTorso,
                     const utility::input::LimbID& leg,
                     const message::input::Sensors& sensors);
    };
}  // namespace motion
}  // namespace utility

#endif
