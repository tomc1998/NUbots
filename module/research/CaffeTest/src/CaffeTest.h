#ifndef MODULE_RESEARCH_CAFFETEST_H
#define MODULE_RESEARCH_CAFFETEST_H

#include <nuclear>

namespace module {
namespace research {

    class CaffeTest : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the CaffeTest reactor.
        explicit CaffeTest(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_RESEARCH_CAFFETEST_H
