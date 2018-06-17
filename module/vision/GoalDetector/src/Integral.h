
#pragma once

#include <algorithm>  // req'd for std::min/max
#include <nuclear>
#include "GoalMatcherConstants.h"
#include "message/vision/ClassifiedImage.h"

//! Computes the 1d integral image of the specified horizon line
//! in image img.  Assumes source image to be a 32-bit floating point.  Returns IplImage in 32-bit float form.
void Integral(std::unique_ptr<std::vector<float>>& data,
              std::shared_ptr<const message::vision::ClassifiedImage> frame_3,
              std::vector<double> upperPolyCoeff,
              std::vector<double> lowerPolyCoeff);

// Convert horizon of image to single channel 32F
void getGrayHorizon(std::unique_ptr<std::vector<float>>& result,
                    std::shared_ptr<const message::vision::ClassifiedImage> frame_4,
                    std::vector<double> upperPolyCoeff,
                    std::vector<double> lowerPolyCoeff);


//! Computes the sum of pixels within the row specified by the left start
//! co-ordinate and width
inline float BoxIntegral(std::unique_ptr<std::vector<float>>& data, int col, int cols) {

    // The subtraction by one for col because col is inclusive.
    int c1 = std::min(col, IMAGE_WIDTH / SURF_SUBSAMPLE) - 1;
    int c2 = std::min(col + cols, IMAGE_WIDTH / SURF_SUBSAMPLE) - 1;

    float A(0.0f), B(0.0f);
    if (c2 >= 0) A = data->at(c2);
    if (c1 >= 0) B = data->at(c1);

    return std::max(0.f, A - B);
}
