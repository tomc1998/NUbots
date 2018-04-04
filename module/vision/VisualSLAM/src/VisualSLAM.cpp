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
#include "opencv2/imgcodecs/imgcodecs.hpp"
//#include "utility/vision/fourcc.h"
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
            imageWidth  = config["imageWidth"].as<uint>();
            imageHeight = config["imageHeight"].as<uint>();
            strSettingsFile = config["strSettingsFile"].as<std::string>();
            strVocFile = config["strVocFile"].as<std::string>();
            cameraCombination = static_cast<System::eSensor>(config["cameraCombination"].as<int>());
        });

        on<Startup>().then([this] {

            SLAM.Initialize(strVocFile,strSettingsFile,cameraCombination);
            auto pLaunchLocalMapping = std::make_unique<LaunchMapping>();
            auto pLaunchLoopClosing  = std::make_unique<LaunchLoopClosing>();
            auto pLaunchImageLoading = std::make_unique<LaunchImageLoading>();
            emit(std::move(pLaunchLocalMapping));
            emit(std::move(pLaunchLoopClosing));
            emit(std::move(pLaunchImageLoading));

            // Loading in timestamp and image filepath data
            //LoadImages("/home/vagrant/Datasets/NUbotsRoom_Dataset1/", vstrImageFilenames, vTimestamps);
            //LoadImages("/home/vagrant/Datasets/LoungeRoom_Dataset2/", vstrImageFilenames, vTimestamps);
            //nImages = vstrImageFilenames.size();


            // Camera parameters when running from desktop
            /*
            auto cameraParameters = std::make_unique<CameraParameters>();

            // Generic camera parameters
            cameraParameters->imageSizePixels << 1280,1024;

            // Radial specific
            cameraParameters->lens                   = CameraParameters::LensType::RADIAL;
            cameraParameters->radial.radiansPerPixel = 0.0025527;
            cameraParameters->centreOffset           << 397,288;

            emit(std::move(cameraParameters));
            */
            std::cout << "Setup Finished" << std::endl;
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
                    std::vector<uint8_t> data(imageHeight * imageWidth, 0);
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
                    image->dimensions = {imageWidth, imageHeight};
                    image->data       = data;
                    image->camera_id  = 1;

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
        /*
        // Emitting Images as fast as they can be processed
        on<Trigger<LaunchImageLoading>>().then([this]()
        {
            process_loop_start_time = NUClear::clock::now();
            for (curImage = 0; curImage < nImages; curImage++) {
                // Load image
                cv::Mat raw_image;
                raw_image = cv::imread(vstrImageFilenames[curImage], CV_LOAD_IMAGE_UNCHANGED);

                // Converting matrix to a concatenated long vector
                std::vector<uint8_t> data(imageHeight * imageWidth, 0);
                if (!raw_image.data)
                    std::cout << "Could not open or find the image at " << vstrImageFilenames[curImage] << std::endl;
                else 
                    data.assign(raw_image.datastart, raw_image.dataend);

                // Create the image 
                auto image = std::make_unique<Image>();
                image->dimensions = {imageWidth, imageHeight};
                image->data       = data;
                image->timestamp  = NUClear::clock::now();
                image->camera_id  = 1;

                std::cout << "Emitting Image #" << curImage << "/" << nImages <<std::endl;
                emit<Scope::DIRECT>(std::move(image));
            }
            process_loop_end_time = NUClear::clock::now();
            // Stop all threads
            SLAM.Shutdown();

            //Calculate Average Frame Rate
            double process_loop_timer = std::chrono::duration_cast<std::chrono::duration<double>>(
                         process_loop_end_time - process_loop_start_time)
                         .count();
            double processLoopTimerAvg = process_loop_timer / nImages;
            double frameRate = 1 / processLoopTimerAvg;
            std::cout << "Frame rate: " << frameRate << std::endl;

            // Save camera trajectory
            SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

        });
        */

        // Tracking thread
        on<Trigger<Image>, Single>().then([this]
            (const Image& newImage) {

            if (newImage.camera_id == 1) // Just using one of the cameras
            {

                curFrame++;
                std::cout << "Received Image #" << curFrame << " from camera ID: " << newImage.camera_id << std::endl;
                // Converting image.data from vector to matrix
                // Justification for using const_cast: 
                // I need to copy across the data from const Image& newImage into a cv::Mat.
                // OpenCV won't let me create a Mat from a const source because the newly created Mat points to the original  
                // data, and it cannot guarantee it won't modify the data. I need to use const_cast to create the Mat, so 
                // I const_cast the Mat at it's creation to ensure I don't modify it. I then clone to a new Mat which  
                // does copy the data so I can modify it safely.
                const cv::Mat im_const(newImage.dimensions[1],newImage.dimensions[0],CV_8UC1,const_cast<uint8_t*>(newImage.data.data()));
                cv::Mat im_bayer = im_const.clone();

                // Converting image from bayer to grayscale
                cv::Mat im_rgb;
                cv::Mat im_gray;
                cv::cvtColor(im_bayer,im_rgb,cv::COLOR_BayerBG2RGB);
                cv::cvtColor(im_rgb,im_gray,cv::COLOR_RGB2GRAY);

                // Cropping image to allow opencv calibration to work properly
                cv::Rect myROI(256, 205, 768, 614);

                
                
                //std::string strPathSaveImageGRAY = "iGusGrayImage" + std::to_string(curFrame) +".jpg";
                //cv::imwrite(strPathSaveImageGRAY,im_gray);
                
                // Process Image
                cv::Mat Tcw = SLAM.TrackMonocular(im_gray(myROI),newImage.timestamp.time_since_epoch().count());

                

                // Emit Transform
                auto transformCW = make_unique<Transform_CW>();
                transformCW->Tcw = Tcw;
                emit(std::move(transformCW));

                if (curFrame  == 1500)
                {
                    process_loop_end_time = NUClear::clock::now();
                    SLAM.Shutdown();

                    //Calculate Average Frame Rate
                    double process_loop_timer = std::chrono::duration_cast<std::chrono::duration<double>>(
                             process_loop_end_time - process_loop_start_time)
                             .count();
                    double processLoopTimerAvg = process_loop_timer / curFrame;
                    double frameRate = 1 / processLoopTimerAvg;
                    std::cout << "Frame rate: " << frameRate << std::endl;
                    // Save camera trajectory
                    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
                }
            }
        });

        // Mapping thread
        on<Trigger<LaunchMapping>>().then([this]()
        {
            std::cout << "Mapping Launched" << endl;
            SLAM.launchLocalMapping();
            process_loop_start_time = NUClear::clock::now();
        });
        
        // Loop Closing thread
        on<Trigger<LaunchLoopClosing>>().then([this]()
        {
            SLAM.launchLoopClosing();
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
