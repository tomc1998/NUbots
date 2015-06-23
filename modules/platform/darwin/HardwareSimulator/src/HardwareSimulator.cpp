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
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "HardwareSimulator.h"

#include <armadillo>

#include "messages/motion/ServoTarget.h"
#include "messages/platform/darwin/DarwinSensors.h"
#include "messages/input/ServoID.h"
#include "utility/math/angle.h"
#include "messages/support/Configuration.h" 

namespace modules {
namespace platform {
namespace darwin {

    using messages::platform::darwin::DarwinSensors;
    using messages::motion::ServoTarget;
    using messages::input::ServoID;
    using messages::support::Configuration;

    HardwareSimulator::HardwareSimulator(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        /*
         CM730 Data
         */
        //Read our Error code
        sensors.cm730ErrorFlags = 0;

        // LED Panel
        sensors.ledPanel.led2 = 0;
        sensors.ledPanel.led3 = 0;
        sensors.ledPanel.led4 = 0;

        // Head LED
        sensors.headLED.r = 0;
        sensors.headLED.g = 0;
        sensors.headLED.b = 0;

        // Head LED
        sensors.eyeLED.r = 0;
        sensors.eyeLED.g = 0;
        sensors.eyeLED.b = 0;

        // Buttons
        sensors.buttons.left = 0;
        sensors.buttons.middle = 0;

        // Voltage (in volts)
        sensors.voltage = 0;

        // Accelerometer (in m/s^2)
        sensors.accelerometer.x = 0;
        sensors.accelerometer.y = 0;
        sensors.accelerometer.z = 9.8;

        // Gyroscope (in radians/second)
        sensors.gyroscope.x = 0;
        sensors.gyroscope.y = 0;
        sensors.gyroscope.z = 0;

        /*
         Force Sensitive Resistor Data
         */

        // Right Sensor
        // Error
        sensors.fsr.right.errorFlags = 0;

        // Sensors
        sensors.fsr.right.fsr1 = 1;
        sensors.fsr.right.fsr2 = 1;
        sensors.fsr.right.fsr3 = 1;
        sensors.fsr.right.fsr4 = 1;

        // Centre
        sensors.fsr.right.centreX = 0;
        sensors.fsr.right.centreY = 0;

        // Left Sensor
        // Error
        sensors.fsr.left.errorFlags = 0;

        // Sensors
        sensors.fsr.left.fsr1 = 1;
        sensors.fsr.left.fsr2 = 1;
        sensors.fsr.left.fsr3 = 1;
        sensors.fsr.left.fsr4 = 1;

        // Centre
        sensors.fsr.left.centreX = 0;
        sensors.fsr.left.centreY = 0;

        /*
         Servos
         */

        for (int i = 0; i < 20; ++i) {
            // Get a reference to the servo we are populating
            DarwinSensors::Servo& servo = sensors.servo[i];

            // Error code
            servo.errorFlags = 0;

            // Booleans
            servo.torqueEnabled = true;

            // Gain
            servo.dGain = 0;
            servo.iGain = 0;
            servo.pGain = 0;

            //Torque
            servo.torque = 0;

            // Targets
            servo.goalPosition = 0;
            servo.movingSpeed = M_PI_4;

            // Present Data
            servo.presentPosition = 0;
            servo.presentSpeed = 0;
            servo.load = 0;

            // Diagnostic Information
            servo.voltage = 0;
            servo.temperature = 0;
        }

        on<Trigger<Configuration<HardwareSimulator>>>("Hardware Simulator Config",[this](const Configuration<HardwareSimulator>& config){
            imu_drift_rate = config["imu_drift_rate"].as<float>();

            noise.accelerometer.x = config["noise"]["accelerometer"]["x"].as<float>();
            noise.accelerometer.y = config["noise"]["accelerometer"]["y"].as<float>();
            noise.accelerometer.z = config["noise"]["accelerometer"]["z"].as<float>();

            noise.gyroscope.x = config["noise"]["gyroscope"]["x"].as<float>();
            noise.gyroscope.y = config["noise"]["gyroscope"]["y"].as<float>();
            noise.gyroscope.z = config["noise"]["gyroscope"]["z"].as<float>();

        });

        on<Trigger<DarwinSensors::Gyroscope>>("Receive Simulated Gyroscope", [this](const DarwinSensors::Gyroscope& gyro){
            gyroQueue.push(gyro);
        });


        on<Trigger<Every<UPDATE_FREQUENCY, Per<std::chrono::seconds>>>, Options<Single>>([this](const time_t&) {

            for (int i = 0; i < 20; ++i) {

                auto& servo = sensors.servo[i];
                float movingSpeed = servo.movingSpeed == 0 ? 0.1 : servo.movingSpeed / 60;
                movingSpeed = movingSpeed > 0.1 ? 0.1 : movingSpeed;


                if (std::abs(servo.presentPosition - servo.goalPosition) < movingSpeed) {
                    servo.presentPosition = servo.goalPosition;
                }
                else {
                    arma::vec3 present = { cos(servo.presentPosition), sin(servo.presentPosition), 0 };
                    arma::vec3 goal = { cos(servo.goalPosition), sin(servo.goalPosition), 0 };

                    arma::vec3 cross = arma::cross(present, goal);
                    if(cross[2] > 0) {
                        servo.presentPosition = utility::math::angle::normalizeAngle(servo.presentPosition + movingSpeed);
                    }
                    else {
                        servo.presentPosition = utility::math::angle::normalizeAngle(servo.presentPosition - movingSpeed);
                    }
                }
            }

            // Gyro:
            // Note: This reaction is not (and should not be) synced with the
            // 'Receive Simulated Gyroscope' reaction above, so we can't
            // reliably query the size of the gyroQueue.
            arma::vec3 sumGyro = {0,0,0};
            while (!gyroQueue.empty()){
                auto g = gyroQueue.front();
                sumGyro += arma::vec3({g.x,g.y,g.z});
                gyroQueue.pop();
            }
            sumGyro = (sumGyro * UPDATE_FREQUENCY + arma::vec3({0,0,imu_drift_rate})) ;
            sensors.gyroscope.x = sumGyro[0];
            sensors.gyroscope.y = sumGyro[1];
            sensors.gyroscope.z = sumGyro[2];

            sensors.timestamp = NUClear::clock::now();
            
            // //Debug:
            // integrated_gyroscope += sumGyro + arma::vec3({0,0,imu_drift_rate});
            // std::cout << "HardwareSimulator gyroscope = " << sensors.gyroscope.x << ", " << sensors.gyroscope.y << ", " << sensors.gyroscope.z << std::endl;
            // std::cout << "HardwareSimulator integrated_gyroscope = " << integrated_gyroscope.t() << std::endl;
            
            //Add some noise so that sensor fusion doesnt converge to a singularity
            auto sensors_message = std::make_unique<DarwinSensors>(sensors);
            addNoise(sensors_message);
            // Send our nicely computed sensor data out to the world
            emit(std::move(sensors_message));

        });

        // This trigger writes the servo positions to the hardware
        on<Trigger<std::vector<ServoTarget>>>([this](const std::vector<ServoTarget>& commands) {
            for (auto& command : commands) {

                // Calculate our moving speed
                float diff = utility::math::angle::difference(command.position, sensors.servo[command.id].presentPosition);
                NUClear::clock::duration duration = command.time - NUClear::clock::now();

                float speed;
                if (duration.count() > 0) {
                    speed = diff / (double(duration.count()) / double(NUClear::clock::period::den));
                }
                else {
                    speed = 0;
                }

                // Set our variables
                auto& servo = sensors.servo[command.id];
                servo.movingSpeed = speed;
                servo.goalPosition = command.position;
            }

        });

        on<Trigger<ServoTarget>>([this](const ServoTarget command) {
            auto commandList = std::make_unique<std::vector<ServoTarget>>();
            commandList->push_back(command);

            // Emit it so it's captured by the reaction above
            emit<Scope::DIRECT>(std::move(commandList));
        });
    }

    float centered_noise() {
        return rand() / float(RAND_MAX) - 0.5f;
    }

    void HardwareSimulator::addNoise(std::unique_ptr<DarwinSensors>& sensors){
        // TODO: Use a more standard c++ random generator.
        sensors->accelerometer.x += noise.accelerometer.x * centered_noise();
        sensors->accelerometer.y += noise.accelerometer.y * centered_noise();
        sensors->accelerometer.z += noise.accelerometer.z * centered_noise();

        sensors->gyroscope.x += noise.gyroscope.x * centered_noise();
        sensors->gyroscope.y += noise.gyroscope.y * centered_noise();
        sensors->gyroscope.z += noise.gyroscope.z * centered_noise();
    }
}
}
}