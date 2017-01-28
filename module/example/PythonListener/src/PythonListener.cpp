#include "PythonListener.h"

#include "message/example/PythonResponse.h"

namespace module {
namespace example {

    using message::example::PythonResponse;

    PythonListener::PythonListener(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Trigger<PythonResponse>>().then([this] (const PythonResponse& response) {
            log("Message from python:", response.message);
        });
    }
}
}
