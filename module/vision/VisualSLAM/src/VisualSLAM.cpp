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
#include <fstream>
#include <iomanip>
#include <chrono>
#include <nuclear>

namespace module {
namespace vision {

    using extension::Configuration;
    using message::input::Image;


    VisualSLAM::VisualSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        //on<Configuration>("VisualSLAM.yaml").then([this](const Configuration& config) {
            // Use configuration here from file VisualSLAM.yaml
        //});

        on<Startup>().then([this] {

            // Loading in timestamp and image filepath data
            LoadImages("/home/vagrant/Datasets/NUbotsRoom_Dataset1/", vstrImageFilenames, vTimestamps);
            nImages = vstrImageFilenames.size();
            curImage = 0;
        });

        // Emitting loaded images
        // Loop two times per second
        on<Every<2,std::chrono::seconds>>().then([this] {

            std::cout << "Starting Image Emitting Loop" << std::endl;
            if (curImage < nImages) {
                loop_start_time = NUClear::clock::now();
                loop_start_timestamp = vTimestamps.at(curImage);
                std::chrono::milliseconds halfASecond(500);

                // This is our first image
                if (curImage == 0) {
                    start_time = NUClear::clock::now();
                    first_timecode = vTimestamps.at(0);
                    std::cout << "First image setup. Start_time: " << start_time.time_since_epoch().count() << ", first_timecode: " << first_timecode.count() << std::endl;
                }

                while((vTimestamps.at(curImage)-loop_start_timestamp) < halfASecond) {
                    std::cout << "Looping" << std::endl;
                    // Load image
                    std::cout << "Loading image " << curImage << std::endl;
                    cv::Mat raw_image;
                    raw_image = cv::imread(vstrImageFilenames[curImage], CV_LOAD_IMAGE_UNCHANGED);

                    // Converting matrix to a concatenated long vector
                    std::vector<uint8_t> data(image_height * image_width, 0);
                    if (!raw_image.data) {
                        std::cout << "Could not open or find the image at " << vstrImageFilenames[curImage] << std::endl;
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

                    // Work out when we should emit this packet
                    NUClear::clock::time_point emit_time = start_time + vTimestamps.at(curImage) - first_timecode;

                    emit<Scope::DELAY>(std::move(image),emit_time - NUClear::clock::now());

                    curImage++;
                    
                    // This is our last image
                    if (curImage == nImages)
                        break;
                }
            }
        });

        // Visual Odometry - motion estimation thread
        on<Trigger<Image>, Single>().then([this](const Image& newImage) {

            std:: cout << "Image Received" << std::endl;
            // Sparse Model-based Image Alignment
            SparseImageAlignment sia;
            T_kkminus1 = sia.sparseImageAlignment(newImage);
            // Feature Alignment


            // Pose and Structure Refinement



        });

    }

    void VisualSLAM::LoadImages(const std::string strPathToSequence,
                                std::vector<std::string> &vstrImageFilenames,
                                std::vector<std::chrono::microseconds> &vTimestamps) {
        std::ifstream fTimes;
        std::string strPathTimeFile = strPathToSequence + "TimeStamp.txt";
        fTimes.open(strPathTimeFile.c_str());
        while (!fTimes.eof()) {
            std::string s;
            getline(fTimes, s);
            if (!s.empty()) {
                std::stringstream ss;
                ss << s;
                int t;
                ss >> t; // in milliseconds
                std::chrono::microseconds tt(t*1000);
                vTimestamps.push_back(tt); // in microseconds
            }
        }

        std::string strPrefixLeft = strPathToSequence;

        const int nTimes = vTimestamps.size();
        vstrImageFilenames.resize(nTimes);

        for (int i = 0; i < nTimes; i++) {
            std::stringstream ss;
            ss << std::setfill('0') << std::setw(6) << i;
            vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".jpg";
        }
    }
}
}
