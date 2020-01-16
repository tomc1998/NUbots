#include "LocalisationTest.h"

#include "extension/Configuration.h"

namespace module {
namespace localisation {

    using extension::Configuration;

    LocalisationTest::LocalisationTest(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("LocalisationTest.yaml").then([this] (const Configuration& config) {
            // Use configuration here from file LocalisationTest.yaml
        });
    }
}
}
