#ifndef MODULE_LOCALISATION_ODOMETRYLOG_H
#define MODULE_LOCALISATION_ODOMETRYLOG_H

#include <nuclear>
#include "utility/math/matrix/Transform2D.h"

namespace module {
namespace localisation {

    class OdometryLog : public NUClear::Reactor {
    private:
        utility::math::matrix::Transform2D localisationOffset = {0, 0, 0};
        std::ofstream logFile;
        std::string logFilePath;
    public:
        /// @brief Called by the powerplant to build and setup the OdometryLog reactor.
        explicit OdometryLog(std::unique_ptr<NUClear::Environment> environment);

    };
}  // namespace localisation
}  // namespace module

#endif  // MODULE_LOCALISATION_ODOMETRYLOG_H
