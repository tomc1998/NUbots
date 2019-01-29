#include "CaffeTest.h"

#include <caffe/caffe.hpp>

#include "extension/Configuration.h"

namespace module {
namespace research {

    using extension::Configuration;

    CaffeTest::CaffeTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("CaffeTest.yaml").then([this](const Configuration& config) {
            // Use configuration here from file CaffeTest.yaml
        });

        on<Startup>().then([this] {
            FLAGS_logtostderr     = true;
            FLAGS_alsologtostderr = true;
            FLAGS_stderrthreshold = google::GLOG_INFO;
            ::google::SetLogDestination(google::GLOG_INFO, "/home/nubots/logs");
            ::google::InitGoogleLogging("CaffeTest");
            log<NUClear::INFO>("Caffe found", caffe::Caffe::EnumerateDevices(false), "devices");
        });
    }
}  // namespace research
}  // namespace module
