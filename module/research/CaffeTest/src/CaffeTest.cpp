#include "CaffeTest.h"

#include "extension/Configuration.h"

namespace module {
namespace research {

    using extension::Configuration;

    CaffeTest::CaffeTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("CaffeTest.yaml").then([this](const Configuration& config) {
            // Use configuration here from file CaffeTest.yaml
        });

        on<Startup, With<CameraParameters>>().then([this](const CameraParameters& cam) {
            // On Startup example
        });
    }
}
}
