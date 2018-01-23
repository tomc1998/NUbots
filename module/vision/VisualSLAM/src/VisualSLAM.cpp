/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUbots <nubots@nubots.net>
 */

#include "VisualSLAM.h"

#include "extension/Configuration.h"
#include "SparseImageAlignment.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "utility/vision/fourcc.h"
#include "stdio.h"
#include <iostream>

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::Image;


    VisualSLAM::VisualSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        //on<Configuration>("VisualSLAM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file VisualSLAM.yaml
        //});

        on<Startup>().then([this] {
            // Loading in first image
            cv::Mat raw_image;
            raw_image = cv::imread("/home/vagrant/Datasets/NUbotsRoom_Dataset1/000000.jpg", CV_LOAD_IMAGE_GRAYSCALE);

            // Converting matrix to a concatenated long vector
            std::vector<uint8_t> data(image_height * image_width, 0);
            if (!raw_image.data) {
                std::cout << "Could not open or find the image" << std::endl;
            }
            else if (raw_image.isContinuous()) {
                data.assign(raw_image.datastart, raw_image.dataend);
            }
            else {
                for (int i = 0; i < raw_image.rows; ++i) {
                    data.insert(data.end(), raw_image.ptr<uchar>(i), raw_image.ptr<uchar>(i) + raw_image.cols);
                }
            }

            /* Create the image */
            auto image = std::make_unique<Image>();
            image->format     = utility::vision::FOURCC::GREY;
            image->dimensions = {image_width, image_width};
            image->data       = data;
            emit(std::move(image));
        });

        // Visual Odometry - motion estimation thread
        on<Trigger<Image>, Single>().then([this](const Image& newImage) {

            // Sparse Model-based Image Alignment
            SparseImageAlignment sia;
            T_kkminus1 = sia.sparseImageAlignment(newImage);
            std::cout << "T_kk-1 = " << T_kkminus1 << std::endl;
            // Feature Alignment


            // Pose and Structure Refinement



        });
    }
}
}
