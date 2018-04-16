#include "OdometryLog.h"

#include "extension/Configuration.h"

#include "message/behaviour/Nod.h"
#include "message/input/Sensors.h"
#include "message/localisation/Field.h"
#include "message/platform/darwin/DarwinSensors.h"

#include "utility/math/matrix/Transform3D.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/support/eigen_armadillo.h"
#include "utility/support/yaml_armadillo.h"

#include "utility/input/ServoID.h"

namespace module {
namespace localisation {

    using extension::Configuration;
    using message::input::Sensors;
    using message::localisation::Field;
    using utility::math::matrix::Transform2D;
    using utility::math::matrix::Transform3D;
    using message::platform::darwin::ButtonLeftDown;
    using message::behaviour::Nod;
    using utility::nubugger::graph;
    using namespace std::chrono;
    OdometryLog::OdometryLog(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)),logFile(), logFilePath("odom_data.CSV") {

        on<Configuration>("OdometryLog.yaml").then([this](const Configuration& config) {
            // Use configuration here from file OdometryLog.yaml
            localisationOffset = config["localisationOffset"].as<arma::vec>();
            logFilePath="odom_data.csv";
        });

        on<Startup>().then("Leg Loads Logger Startup", [this]() {
                logFile.open(logFilePath, std::ios::out | std::ios::binary);

                if ((logFile.is_open() == true) && (logFile.good() == true)) {
                    logFile << "Time, x_pos, y_pos, angle" << std::endl;
                            /*
                            << "RightHipPitchPosition, RightHipRollPosition, RightHipYawPosition,"
                            << "LeftHipPitchPosition, LeftHipRollPosition,LeftHipYawPosition, "
                            << "RightKneePosition, LeftKneePosition, "
                            << "RightAnklePitchPosition, RightAnkleRollPosition ,RightAnkleYawPosition,"
                            << "LeftAnklePitchPosition, LeftAnkleRollPosition, LeftAnkleYawPosition "
                            std::endl;
                            */
                }

                else {
                    NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
                }
            });

        on<Trigger<ButtonLeftDown>, Single, With<Sensors>, Sync<OdometryLog>>().then(
            [this](const Sensors& sensors) {
                NUClear::log("Localisation Orientation reset. This direction is now forward.");
                emit(std::make_unique<Nod>(true));
                Transform2D Trw    = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
                localisationOffset = Trw;

                if ((logFile.is_open() == true) && (logFile.good() == true)) {
                    logFile << "Localisation Orientation reset This direction is now forward,"
                            << Trw.x() << "," << Trw.y() << "," << Trw.angle() << std::endl; /*","
                            <<"blank , blank , blank, blank, blank, blank , blank, blank, blank,"
                            << "blank , blank, blank, blank, blank " << std::endl; */

                }
                 else {
                    NUClear::log<NUClear::ERROR>("Failed to open log file '", logFilePath, "'.");
                }
            });


        on<Trigger<Sensors>, Sync<OdometryLog>, Single>().then("Odometry Loc", [this](const Sensors& sensors) {

              // If the file isn't open skip
            if (!logFile.is_open()) {{
                return;
            }}
                /* get current time */
            auto timestamp = std::chrono::high_resolution_clock::now();

                /* get current joint angles */
            /*
            float RightHipPitchPosition   = sensors.servo[ServoID::R_HIP_PITCH].presentPosition;
            float RightHipRollPosition   = sensors.servo[ServoID::R_HIP_ROLL].presentPosition;
            float RightHipYawPosition   = sensors.servo[ServoID::R_HIP_YAW].presentPosition;

            float LeftHipPitchPosition    = sensors.servo[ServoID::L_HIP_PITCH].presentPosition;
            float LeftHipRollPosition    = sensors.servo[ServoID::L_HIP_ROLL].presentPosition;
            float LeftHipYawPosition    = sensors.servo[ServoID::L_HIP_YAW].presentPosition;

            float RightKneePosition       = sensors.servo[ServoID::R_KNEE].presentPosition;
            float LeftKneePosition        = sensors.servo[ServoID::L_KNEE].presentPosition;

            float RightAnklePitchPosition = sensors.servo[ServoID::R_ANKLE_PITCH].presentPosition;
            float RightAnkleRollPosition = sensors.servo[ServoID::R_ANKLE_ROLL].presentPosition;
            float RightAnkleYawPosition = sensors.servo[ServoID::R_ANKLE_YAW].presentPosition;

            float LeftAnklePitchPosition  = sensors.servo[ServoID::L_ANKLE_PITCH].presentPosition;
            float LeftAnkleRollPosition = sensors.servo[ServoID::L_ANKLE_ROLL].presentPosition;
            float LeftAnkleYawPosition = sensors.servo[ServoID::L_ANKLE_YAW].presentPosition;
            */
            /* get current x,y and angle values */
            Transform2D Trw = Transform3D(convert<double, 4, 4>(sensors.world)).projectTo2D();
            Transform2D Twr = Trw.i();

            Transform2D state = localisationOffset.localToWorld(Twr);

            logFile << timestamp.time_since_epoch().count() << "," << Trw.x() << "," << Trw.y() << "," << Trw.angle() << std::endl;/*","
                    << RightHipPitchPosition << "," << RightHipRollPosition << "," << RightHipYawPosition << ","
                    << LeftHipPitchPosition << "," << LeftHipRollPosition << "," << LeftHipYawPosition << ","
                    << RightKneePosition << "," << LeftKneePosition << ","
                    << RightAnklePitchPosition << "," << RightAnkleRollPosition << "," << RightAnkleYawPosition << ","
                    << LeftAnklePitchPosition << "," << LeftAnkleRollPosition << "," << LeftAnkleYawPosition
                    << std::endl;
                    */
            });


        on<Shutdown>().then("OdometryLog Shutdown", [this]() { logFile.close(); });


     } // OdometryLog




}  // namespace localisation
}  // namespace module
