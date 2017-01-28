#include "PeriodicEmitter.h"

#include "extension/Configuration.h"
#include "message/example/ExampleMessage.h"

namespace module {
namespace example {

    using extension::Configuration;
    using message::example::ExampleMessage;

    PeriodicEmitter::PeriodicEmitter(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {

        on<Configuration>("PeriodicEmitter.yaml").then([this] (const Configuration& /*config*/) {
            // Use configuration here from file PeriodicEmitter.yaml
        });

        on<Every<1, std::chrono::seconds>>().then([this] {
            log("Emitting a message!");
            emit(std::make_unique<ExampleMessage>());
        });
    }
}
}
