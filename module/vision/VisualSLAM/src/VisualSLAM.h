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

#ifndef MODULE_VISION_VISUALSLAM_H
#define MODULE_VISION_VISUALSLAM_H

#include <nuclear>
#include "message/input/Image.h"
#include "SVO.h"
#include <string>

namespace module {
namespace vision {

    class VisualSLAM : public NUClear::Reactor {

    public:
        /// @brief Called by the powerplant to build and setup the VisualSLAM reactor.
        explicit VisualSLAM(std::unique_ptr<NUClear::Environment> environment);

        void LoadImages(const std::string strPathToSequence,
                    	std::vector<std::string> &vstrImageFilenames,
                    	std::vector<std::chrono::microseconds> &vTimestamps);

    private:

        SVO svo;
        std::vector<std::string> vstrImageFilenames;
    		std::vector<std::chrono::microseconds> vTimestamps; //in microseconds

    		// Number of images to process
    		int nImages;

    		// Current image number we are processing
    		int curImage;

  		  // When we started playing this file
        NUClear::clock::time_point start_time;

        // When we started this loop
        NUClear::clock::time_point loop_start_time;

        // When we started this loop
        std::chrono::microseconds loop_start_timestamp;

        // The first time that appears in the file
        std::chrono::microseconds first_timecode;
    		uint IMAGE_WIDTH;
    		uint IMAGE_HEIGHT;

    };
}
}

#endif  // MODULE_VISION_VISUALSLAM_H
