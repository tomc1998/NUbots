/*
 * This file is part of Darwin Sensor Filter.
 *
 * Darwin Sensor Filter is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Darwin Sensor Filter is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Darwin Sensor Filter.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "SensorFilter.h"

#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/Sensors.h"
#include "messages/support/Configuration.h"
#include "utility/nubugger/NUgraph.h"
#include "utility/math/matrix.h"
#include "utility/motion/ForwardKinematics.h"
#include "utility/motion/RobotModels.h"

namespace modules {
    namespace platform {
        namespace darwin {

            using messages::support::Configuration;
            using messages::platform::darwin::DarwinSensors;
            using messages::input::Sensors;
            using utility::nubugger::graph;
            using messages::input::ServoID;
            using utility::motion::kinematics::calculateAllPositions;
            using utility::motion::kinematics::DarwinModel;
            using utility::motion::kinematics::calculateCentreOfMass;

            SensorFilter::SensorFilter(std::unique_ptr<NUClear::Environment> environment)
            : Reactor(std::move(environment))
            , orientationFilter(arma::vec({0,0,-1,1,0,0}))
            , velocityFilter(arma::vec3({0,0,0}))
            , lastOrientationMatrix(arma::eye(3,3))
            , previousMeasuredTorsoFromLeftFoot()
            , previousMeasuredTorsoFromRightFoot() {

                on<Trigger<Configuration<SensorFilter>>>([this](const Configuration<SensorFilter>& file){
                    DEFAULT_NOISE_GAIN = file.config["DEFAULT_NOISE_GAIN"];
                    HIGH_NOISE_THRESHOLD = file.config["HIGH_NOISE_THRESHOLD"];
                    HIGH_NOISE_GAIN = file.config["HIGH_NOISE_GAIN"];
                    LOW_NOISE_THRESHOLD = file.config["LOW_NOISE_THRESHOLD"];

                    SUPPORT_FOOT_FSR_THRESHOLD = file.config["SUPPORT_FOOT_FSR_THRESHOLD"];
                    REQUIRED_NUMBER_OF_FSRS = file.config["REQUIRED_NUMBER_OF_FSRS"];
                });

                on<Trigger<DarwinSensors>, Options<Single>>([this](const DarwinSensors& input) {

                    auto sensors = std::make_unique<Sensors>();

                    sensors->timestamp = input.timestamp;

                    if (input.cm730ErrorFlags == DarwinSensors::Error::OK) {
                        std::stringstream s;
                        s << "Error on CM730:";

                        if(input.cm730ErrorFlags & DarwinSensors::Error::INPUT_VOLTAGE) {
                            s << " Input Voltage ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::ANGLE_LIMIT) {
                            s << " Angle Limit ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERHEATING) {
                            s << " Overheating ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::OVERLOAD) {
                            s << " Overloaded ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::INSTRUCTION) {
                            s << " Bad Instruction ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::CORRUPT_DATA) {
                            s << " Corrupt Data ";
                        }
                        if(input.cm730ErrorFlags & DarwinSensors::Error::TIMEOUT) {
                            s << " Timeout ";
                        }

                        NUClear::log<NUClear::WARN>(s.str());
                    }


                    for(uint i = 0; i < 20; ++i) {
                        auto& original = input.servo[i];
                        auto& error = input.servo[i].errorFlags;

                        if(error != DarwinSensors::Error::OK) {
                            std::stringstream s;
                            s << "Error on Servo " << (i + 1) << ":";

                            if(error & DarwinSensors::Error::INPUT_VOLTAGE) {
                                s << " Input Voltage ";
                            }
                            if(error & DarwinSensors::Error::ANGLE_LIMIT) {
                                s << " Angle Limit ";
                            }
                            if(error & DarwinSensors::Error::OVERHEATING) {
                                s << " Overheating ";
                            }
                            if(error & DarwinSensors::Error::OVERLOAD) {
                                s << " Overloaded ";
                            }
                            if(error & DarwinSensors::Error::INSTRUCTION) {
                                s << " Bad Instruction ";
                            }
                            if(error & DarwinSensors::Error::CORRUPT_DATA) {
                                s << " Corrupt Data ";
                            }
                            if(error & DarwinSensors::Error::TIMEOUT) {
                                s << " Timeout ";
                            }

                            NUClear::log<NUClear::WARN>(s.str());
                        }

                        sensors->servos.push_back({
                            original.errorFlags,
                            static_cast<ServoID>(i),
                            original.torqueEnabled,
                            original.pGain,
                            original.iGain,
                            original.dGain,
                            original.goalPosition,
                            original.movingSpeed,
                            original.torqueLimit,
                            original.presentPosition,
                            original.presentSpeed,
                            original.load,
                            original.voltage,
                            float(original.temperature)
                        });
                    }

                    sensors->accelerometer = {-input.accelerometer.y, input.accelerometer.x, -input.accelerometer.z};
                    sensors->gyroscope = {-input.gyroscope.x, -input.gyroscope.y, input.gyroscope.z};

                    /************************************************
                     *                 Orientation                  *
                     ************************************************/
                    double deltaT = (lastUpdate - input.timestamp).count() / double(NUClear::clock::period::den);
                    lastUpdate = input.timestamp;
                    orientationFilter.timeUpdate(deltaT, sensors->gyroscope);

                    arma::mat observationNoise = arma::eye(3,3) * DEFAULT_NOISE_GAIN;
                    double normAcc = std::abs(arma::norm(sensors->accelerometer,2) - 9.807);

                    if(normAcc > HIGH_NOISE_THRESHOLD){
                        observationNoise *= HIGH_NOISE_GAIN;
                    } else if(normAcc > LOW_NOISE_THRESHOLD){
                        observationNoise = arma::eye(3,3) * (HIGH_NOISE_GAIN - DEFAULT_NOISE_GAIN) * (normAcc - LOW_NOISE_THRESHOLD) / (HIGH_NOISE_THRESHOLD - LOW_NOISE_THRESHOLD);
                    }

                    float quality = orientationFilter.measurementUpdate(sensors->accelerometer, observationNoise);
                    arma::vec orientation = orientationFilter.get();
                    sensors->orientation.col(2) = -orientation.rows(0,2);
                    sensors->orientation.col(0) = orientation.rows(3,5);
                    sensors->orientation.col(1) = arma::cross(sensors->orientation.col(2), sensors->orientation.col(0));

                    /************************************************
                     *                  Kinematics                  *
                     ************************************************/
                    sensors->forwardKinematics = calculateAllPositions<DarwinModel>(*sensors);

                    /************************************************
                     *                   Odometry                   *
                     ************************************************/

                    //ODOMETRY
                    //Check support foot:
                    sensors->leftFootDown = false;
                    sensors->rightFootDown = false;
                    sensors->torsoVelocity = {0,0,0};

                    int zeroSensorsLeft = (input.fsr.left.fsr1 == 0) + (input.fsr.left.fsr2 == 0) + (input.fsr.left.fsr3 == 0) + (input.fsr.left.fsr4 == 0);
                    int zeroSensorsRight = (input.fsr.right.fsr1 == 0) + (input.fsr.right.fsr2 == 0) + (input.fsr.right.fsr3 == 0) + (input.fsr.right.fsr4 == 0);

                    if(input.fsr.left.fsr1 + input.fsr.left.fsr2 + input.fsr.left.fsr3 + input.fsr.left.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsLeft <= 4 - REQUIRED_NUMBER_OF_FSRS){
                        sensors->leftFootDown = true;
                    }
                    if(input.fsr.right.fsr1 + input.fsr.right.fsr2 + input.fsr.right.fsr3 + input.fsr.right.fsr4 > SUPPORT_FOOT_FSR_THRESHOLD && zeroSensorsRight <= 4 - REQUIRED_NUMBER_OF_FSRS){
                        sensors->rightFootDown = true;
                    }


                    if(sensors->leftFootDown || sensors->rightFootDown){
                        //velocityFilter.timeUpdate(deltaT, sensors->accelerometer - sensors->orientation.col(2) * 9.807);
                        arma::vec3 measuredTorsoFromLeftFoot = -sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].submat(0,0,2,2).t() * sensors->forwardKinematics[ServoID::L_ANKLE_ROLL].col(3).rows(0,2);
                        arma::vec3 measuredTorsoFromRightFoot = -sensors->forwardKinematics[ServoID::R_ANKLE_ROLL].submat(0,0,2,2).t() * sensors->forwardKinematics[ServoID::R_ANKLE_ROLL].col(3).rows(0,2);

                        arma::vec3 torsoVelFromLeftFoot =  -(measuredTorsoFromLeftFoot - previousMeasuredTorsoFromLeftFoot)/deltaT;//negate hack
                        arma::vec3 torsoVelFromRightFoot =  -(measuredTorsoFromRightFoot - previousMeasuredTorsoFromRightFoot)/deltaT;

                        arma::vec3 averageVelocity = (torsoVelFromLeftFoot * static_cast<int>(sensors->leftFootDown) + torsoVelFromRightFoot * static_cast<int>(sensors->rightFootDown))/(static_cast<int>(sensors->rightFootDown) + static_cast<int>(sensors->leftFootDown));
                        previousMeasuredTorsoFromLeftFoot = measuredTorsoFromLeftFoot;
                        previousMeasuredTorsoFromRightFoot = measuredTorsoFromRightFoot;
                        //FILTER WITH ACCELEROMETER
                        //velocityFilter.measurementUpdate(averageVelocity, arma::eye(3,3) * 1e-6);
                        sensors->torsoVelocity = averageVelocity;
                        //sensors->torsoVelocity = velocityFilter.get();
                    }
                    //END ODOMETRY


                    //MASS MODEL
                    sensors->centreOfMass = calculateCentreOfMass<DarwinModel>(sensors->forwardKinematics, true);
                    //END MASS MODEL

                    /*emit(graph("Filtered Gravity Vector",
                            float(orientation[0]*9.807),
                            float(orientation[1]*9.807),
                            float(orientation[2]*9.807)
                        ));
                     emit(graph("Filtered Forward Vector",
                            float(orientation[3]),
                            float(orientation[4]),
                            float(orientation[5])
                        ));
                    emit(graph("Orientation Quality", quality
                        ));
                    emit(graph("Difference from gravity", normAcc
                        ));
                    emit(graph("Gyro Filtered", sensors->gyroscope[0],sensors->gyroscope[1], sensors->gyroscope[2]
                        ));*/
                    emit(graph("L FSR", input.fsr.left.fsr1, input.fsr.left.fsr2, input.fsr.left.fsr3, input.fsr.left.fsr4
                        ));
                    emit(graph("R FSR", input.fsr.right.fsr1, input.fsr.right.fsr2, input.fsr.right.fsr3, input.fsr.right.fsr4
                        ));
                    emit(graph("Torso Velocity", sensors->torsoVelocity[0], sensors->torsoVelocity[1], sensors->torsoVelocity[2]
                        ));
                    emit(graph("COM", sensors->centreOfMass[0], sensors->centreOfMass[1], sensors->centreOfMass[2], sensors->centreOfMass[3]
                        ));


                    lastOrientationMatrix = sensors->orientation;

                    emit(std::move(sensors));
                });
            }

        }  // darwin
    }  // platform
}  // modules
