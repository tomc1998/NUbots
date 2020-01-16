#ifndef MODULE_LOCALISATION_LOCALISATIONTEST_H
#define MODULE_LOCALISATION_LOCALISATIONTEST_H

#include <nuclear>

namespace module {
namespace localisation {

    class LocalisationTest : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the LocalisationTest reactor.
        explicit LocalisationTest(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_LOCALISATION_LOCALISATIONTEST_H
