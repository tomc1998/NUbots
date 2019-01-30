#include "CaffeTest.h"
#include <CL/cl.h>
#include <fstream>
#include <iostream>
#include "extension/Configuration.h"

namespace module {
namespace research {

    using extension::Configuration;
    using std::cout;
    using std::endl;

    CaffeTest::CaffeTest(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("CaffeTest.yaml").then([this](const Configuration& config) {
            model_file   = config["model_file"].as<std::string>();
            trained_file = config["trained_file"].as<std::string>();
            mean_file    = config["mean_file"].as<std::string>();
            label_file   = config["label_file"].as<std::string>();
            // classifier(model_file, trained_file, mean_file, label_file);

            image_file = config["image_file"].as<std::string>();
        });

        /*
        on<Startup, With<CameraParameters>>().then([this](const CameraParameters& cam) {
            // On Startup example
        });
        */
    }
}
}
