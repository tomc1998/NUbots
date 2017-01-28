#ifndef MODULE_EXAMPLE_PERIODICEMITTER_H
#define MODULE_EXAMPLE_PERIODICEMITTER_H

#include <nuclear>

namespace module {
namespace example {

    class PeriodicEmitter : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PeriodicEmitter reactor.
        explicit PeriodicEmitter(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_EXAMPLE_PERIODICEMITTER_H
