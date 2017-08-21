
#include <iostream>
#include <sstream>
#include <string>
#include "GoalMatcherConstants.h"
#include "message/input/Image.h"
#include "message/localisation/Field.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "utility/vision/fourcc.h"

using namespace cv;
using namespace std;
using message::input::Image;

Image testImageGen(uint8_t imageNum, std::unique_ptr<message::localisation::Field>& field, double* d, int awayImages) {
    vector<double> d_values;
    string folder;
    if (awayImages == 0) {  // check image d values (HOME)
        d_values = {20.0, 15.0, 12.0, 12.0, 15.0, 15.0, 15.0, 15.0, 15.0, 20.0, 15.0, 10.0, 10.0, 10.0, 12.0, 7.0, 5.0,
                    10.0, 7.0,  76.0, 73.0, 71.0, 68.0, 68.0, 71.0, 77.0, 72.0, 72.0, 67.0, 69.0, 69.0, 70.0, 69.0};
        folder             = "/home/vagrant/NUbots/module/vision/GoalDetector/data/check";
        field->position[2] = 0;  // theta
    }
    else {  // test image d values (AWAY)
        d_values = {30.0, 28.0, 12.0, 9.0,  5.0,  10.0, 10.0, 30.0, 40.0, 33.0, 30.0,
                    30.0, 25.0, 26.0, 25.0, 33.0, 25.0, 38.0, 43.0, 37.0, 35.0, 36.0,
                    44.0, 38.0, 38.0, 46.0, 45.0, 37.0, 37.0, 34.0, 33.0, 36.0, 33.0};
        folder             = "/home/vagrant/NUbots/module/vision/GoalDetector/data/test";
        field->position[2] = M_PI;  // theta
    }

    *d = d_values.at(imageNum - 1);
    printf("d is %.0f\n", *d);
    field->position[0] = 0;  // x position
    field->position[1] = 0;  // y position

    string suffix = ".png";
    stringstream ss;
    ss << (int) imageNum;
    std::string number = ss.str();
    string name        = folder + number + suffix;
    cout << "File name is: " << name << endl;

    // Reading in image
    Mat raw_image;
    raw_image = imread(name, CV_LOAD_IMAGE_GRAYSCALE);

    // Converting matrix to a concatenated long vector
    std::vector<uint8_t> data(IMAGE_HEIGHT * IMAGE_WIDTH, 0);
    if (!raw_image.data) {
        cout << "Could not open or find the image" << std::endl;
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
    Image image;
    image.format     = utility::vision::FOURCC::GREY;
    image.dimensions = {IMAGE_WIDTH, IMAGE_HEIGHT};
    image.data       = data;
    return image;
}
