

#include "Integral.h"
#include <stdio.h>
#include "message/input/Image.h"
#include "utility/vision/Vision.h"
#include "utility/vision/fourcc.h"

// Convert horizon of image to single channel 32F
void getGrayHorizon(std::unique_ptr<std::vector<float>>& result,
                    std::shared_ptr<const message::vision::ClassifiedImage> frame_4,
                    std::vector<double> upperPolyCoeff,
                    std::vector<double> lowerPolyCoeff) {

    const auto& image = *frame_4;
    int x0            = 140;
    int x1            = 1230;
    int x             = x0;
    int yUpper;
    int yLower;
    int yDiff;
    int vsum;

    while (x <= x1) {
        // Get pixel sum
        result->at(x0 / SURF_SUBSAMPLE) = 0.0;
        vsum                            = 0;
        yUpper = static_cast<int>(round(upperPolyCoeff[0] * x * x + upperPolyCoeff[1] * x + upperPolyCoeff[2]));
        yLower = static_cast<int>(round(lowerPolyCoeff[0] * x * x + lowerPolyCoeff[1] * x + lowerPolyCoeff[2]));
        yDiff  = yLower - yUpper;
        for (int j = yUpper; j <= yLower; j++) {
            if ((yUpper + j) >= 0 && (yUpper + j) < IMAGE_HEIGHT) {

                utility::vision::Pixel pixel =
                    utility::vision::getPixel(x,
                                              yUpper + j,
                                              image.image->dimensions[0],
                                              image.image->dimensions[1],
                                              image.image->data,
                                              static_cast<utility::vision::FOURCC>(image.image->format));
                vsum += pixel.components.y;
            }
        }
        result->at(x0 / SURF_SUBSAMPLE) = static_cast<float>(vsum / yDiff);

        x += SURF_SUBSAMPLE;
    }
}


//! Computes the 1d integral image of the specified horizon line in 32-bit grey float
void Integral(std::unique_ptr<std::vector<float>>& data,
              std::shared_ptr<const message::vision::ClassifiedImage> frame_3,
              std::vector<double> upperPolyCoeff,
              std::vector<double> lowerPolyCoeff) {

    // convert the image to single channel 32f
    getGrayHorizon(data, frame_3, upperPolyCoeff, lowerPolyCoeff);


    // one row only
    float rs = 0.0f;
    for (int j = 0; j < IMAGE_WIDTH / SURF_SUBSAMPLE; j++) {
        rs += data->at(j);
        data->at(j) = rs;
    }
}
