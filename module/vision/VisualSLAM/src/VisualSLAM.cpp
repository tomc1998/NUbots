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

#include "message/input/CameraParameters.h"
#include "extension/Configuration.h"
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
    using message::input::CameraParameters;


    VisualSLAM::VisualSLAM(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("VisualSLAM.yaml").then([this](const Configuration& config) { 
            IMAGE_WIDTH  = config["image_width"].as<uint>();
            IMAGE_HEIGHT = config["image_height"].as<uint>();
        });

        on<Startup>().then([this] {
            // Loading in timestamp and image filepath data
            LoadImages("/home/vagrant/Datasets/NUbotsRoom_Dataset1/", vstrImageFilenames, vTimestamps);
            nImages = vstrImageFilenames.size();

            // Camera parameters when running from desktop
            auto cameraParameters = std::make_unique<CameraParameters>();

            // Generic camera parameters
            cameraParameters->imageSizePixels << 1280,1024;

            // Radial specific
            cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
            cameraParameters->radial.radiansPerPixel = 0.0025527;
            cameraParameters->centreOffset           << 397,288;

            emit(std::move(cameraParameters));


            for (curImage = 0; curImage < nImages; curImage++) {
                // Load image
                cv::Mat raw_image;
                raw_image = cv::imread(vstrImageFilenames[curImage], CV_LOAD_IMAGE_UNCHANGED);

                // Converting matrix to a concatenated long vector
                std::vector<uint8_t> data(IMAGE_HEIGHT * IMAGE_WIDTH, 0);
                if (!raw_image.data)
                    std::cout << "Could not open or find the image at " << vstrImageFilenames[curImage] << std::endl;
                else 
                    data.assign(raw_image.datastart, raw_image.dataend);

                // Create the image 
                auto image = std::make_unique<Image>();
                image->format     = utility::vision::FOURCC::GREY;
                image->dimensions = {IMAGE_WIDTH, IMAGE_HEIGHT};
                image->data       = data;

                std::cout << "Emitting Image #" << curImage << "/" << nImages <<std::endl;
                emit<Scope::DIRECT>(std::move(image));
            }
        });

        // Emitting loaded images at the correct time
        /*
        on<Every<500,std::chrono::milliseconds>>().then([this] {

            if (curImage < nImages) {
                loop_start_time = NUClear::clock::now();
                loop_start_timestamp = vTimestamps.at(curImage);
                std::chrono::milliseconds halfASecond(500);

                while((vTimestamps.at(curImage)-loop_start_timestamp) < halfASecond) {
                    // Load image
                    cv::Mat raw_image;
                    raw_image = cv::imread(vstrImageFilenames[curImage], CV_LOAD_IMAGE_UNCHANGED);

                    // Converting matrix to a concatenated long vector
                    std::vector<uint8_t> data(IMAGE_HEIGHT * IMAGE_WIDTH, 0);
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

                    // Create the image 
                    auto image = std::make_unique<Image>();
                    image->format     = utility::vision::FOURCC::GREY;
                    image->dimensions = {IMAGE_WIDTH, IMAGE_HEIGHT};
                    image->data       = data;

                    // This is our first image
                    if (curImage == 0) {
                        start_time = NUClear::clock::now();
                        first_timecode = vTimestamps.at(0);
                    }

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
        */

        // Visual Odometry - motion estimation thread
        on<Trigger<Image>,With<CameraParameters>, Single>().then([this]
            (const Image& newImage,
             const CameraParameters& cam) {

            svo.visualOdometry(newImage, cam);

            std::cout << "Image finished processing" << std::endl;

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
