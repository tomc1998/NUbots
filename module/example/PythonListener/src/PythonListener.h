#ifndef MODULE_EXAMPLE_PYTHONLISTENER_H
#define MODULE_EXAMPLE_PYTHONLISTENER_H

#include <nuclear>

namespace module {
namespace example {

    class PythonListener : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the PythonListener reactor.
        explicit PythonListener(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_EXAMPLE_PYTHONLISTENER_H
