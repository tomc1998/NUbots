/*
 * This file is part of NUbots Codebase.
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

#ifndef MODULES_VISION_SVO_H
#define MODULES_VISION_SVO_H

#include <nuclear>
#include "message/input/Image.h"
#include "message/input/CameraParameters.h"

namespace module {
namespace vision {

	class SVO{
	public:
		enum Stage {
		    STAGE_PAUSED,
		    STAGE_FIRST_FRAME,
		    STAGE_SECOND_FRAME,
		    STAGE_DEFAULT_FRAME,
		    STAGE_RELOCALIZING
		};
		enum TrackingQuality {
			TRACKING_INSUFFICIENT,
			TRACKING_BAD,
			TRACKING_GOOD
		};
		enum UpdateResult {
			RESULT_NO_KEYFRAME,
			RESULT_IS_KEYFRAME,
			RESULT_FAILURE
		};

		SVO();
		void visualOdometry(const message::input::Image& newImage, const message::input::CameraParameters& cam);


	private:
		UpdateResult processFrame(const message::input::Image& newImage, const message::input::CameraParameters& cam);
		UpdateResult processSecondFrame(const message::input::Image& newImage, const message::input::CameraParameters& cam);
		UpdateResult processFirstFrame(const message::input::Image& newImage, const message::input::CameraParameters& cam);
		UpdateResult relocalizeFrame(const message::input::Image& newImage, const message::input::CameraParameters& cam);

		Stage stage_;
		TrackingQuality tracking_quality_;

		
		

	};
	
}  // namespace vision
}  // namespace module


#endif