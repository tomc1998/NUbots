#ifndef MODULE_RESEARCH_CAFFETEST_H
#define MODULE_RESEARCH_CAFFETEST_H

#include <nuclear>
#include <string>

namespace module {
namespace research {

    class CaffeTest : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the CaffeTest reactor.
        explicit CaffeTest(std::unique_ptr<NUClear::Environment> environment);

    private:
        std::string model_file;
        std::string trained_file;
        std::string mean_file;
        std::string label_file;
        std::string image_file;
    };
}
}

#endif  // MODULE_RESEARCH_CAFFETEST_H
