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

#include "SVO.h"
#include "stdio.h"
#include "utility/math/vision.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace module {
namespace vision {

	using message::input::Image;
	using message::input::CameraParameters;

	SVO::SVO() :
		stage_(STAGE_FIRST_FRAME),
		tracking_quality_(TRACKING_INSUFFICIENT)
	{}


	void SVO::visualOdometry(const Image& newImage, const CameraParameters& cam) {

		NUClear::clock::time_point startVisOdomTime;

		// process frame
		UpdateResult res = RESULT_FAILURE;
		if(stage_ == STAGE_DEFAULT_FRAME)
			res = processFrame(newImage, cam);
		else if(stage_ == STAGE_SECOND_FRAME)
			res = processSecondFrame(newImage, cam);
		else if(stage_ == STAGE_FIRST_FRAME)
			res = processFirstFrame(newImage, cam);
		//else if(stage_ == STAGE_RELOCALIZING)
			//res = relocalizeFrame(SE3(Matrix3d::Identity(), Vector3d::Zero()),
		    //                  map_.getClosestKeyframe(last_frame_));

		NUClear::clock::time_point endVisOdomTime;
		auto VisOdomTime = endVisOdomTime - startVisOdomTime;

	}

	SVO::UpdateResult SVO::processFrame(const Image& newImage, const CameraParameters& cam) {
		//
	}

	SVO::UpdateResult SVO::processSecondFrame(const Image& newImage, const CameraParameters& cam) {
		stage_ = STAGE_DEFAULT_FRAME;
	}

	SVO::UpdateResult SVO::processFirstFrame(const Image& newImage, const CameraParameters& cam) {
		std::vector<cv::KeyPoint> keypointsD;
		std::vector<cv::Mat> descriptor;

		// Justification for using const_cast:
		// I need to copy across the data from const Image& newImage into a cv::Mat. I then need to edit the data.
		// OpenCV won't let me create a Mat from a const source because the newly created Mat points to the original 
		// data, and it cannot guarantee it won't modify the data. I need to use const_cast to create the Mat, so
		// I const_cast the Mat at it's creation to ensure I don't modify it. I then clone to a new Mat which 
		// does copy the data so I can modify it safely.
		const cv::Mat src_const(newImage.dimensions[1],newImage.dimensions[0],CV_8UC1,const_cast<uint8_t*>(newImage.data.data()));
		cv::Mat src = src_const.clone();

		cv::FAST(src,keypointsD,10);
		cv::drawKeypoints(src, keypointsD, src);
		cv::imwrite("keypoints.jpg",src);
		stage_ = STAGE_SECOND_FRAME;
	}

	SVO::UpdateResult SVO::relocalizeFrame(const Image& newImage, const CameraParameters& cam) {
		//
	}

}  // namespace vision
}  // namespace module