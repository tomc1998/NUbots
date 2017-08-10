
#include <iostream>
#include <sstream>
#include <string>
#include "GoalMatcherConstants.h"
#include "message/localisation/Field.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

std::vector<uint8_t> testImageGen(uint8_t imageNum,
                                  std::unique_ptr<message::localisation::Self>& self,
                                  double* d,
                                  int awayImages) {
    vector<double> d_values;
    string folder;
    if (awayImages == 0) {  // check image d values (HOME)
        d_values = {20.0, 15.0, 12.0, 12.0, 15.0, 15.0, 15.0, 15.0, 15.0, 20.0, 15.0, 10.0, 10.0, 10.0, 12.0, 7.0, 5.0,
                    10.0, 7.0,  76.0, 73.0, 71.0, 68.0, 68.0, 71.0, 77.0, 72.0, 72.0, 67.0, 69.0, 69.0, 70.0, 69.0};
        folder = "/home/vagrant/NUbots/module/vision/GoalDetector/data/check";
    }
    else {  // test image d values (AWAY)
        d_values = {30.0, 28.0, 12.0, 9.0,  5.0,  10.0, 10.0, 30.0, 40.0, 33.0, 30.0,
                    30.0, 25.0, 26.0, 25.0, 33.0, 25.0, 38.0, 43.0, 37.0, 35.0, 36.0,
                    44.0, 38.0, 38.0, 46.0, 45.0, 37.0, 37.0, 34.0, 33.0, 36.0, 33.0};
        folder = "/home/vagrant/NUbots/module/vision/GoalDetector/data/test";
    }

    Mat raw_image;
    *d = d_values.at(imageNum - 1);
    printf("d is %.0f\n", *d);
    std::vector<uint8_t> image(IMAGE_HEIGHT * IMAGE_WIDTH * 2, 0);
    self->position[0] = 0;  // x position
    self->position[1] = 0;  // y position
    self->heading[0]  = 1;  // x component heading
    self->heading[1]  = 0;  // y component heading

    string suffix = ".png";
    stringstream ss;
    ss << (int) imageNum;
    std::string number = ss.str();
    string name        = folder + number + suffix;
    cout << "File name is: " << name << endl;

    raw_image = imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    if (!raw_image.data) {
        cout << "Could not open or find the image" << std::endl;
    }
    else {
        printf("raw_image rows: %d, columns: %d\n", raw_image.rows, raw_image.cols);
        for (int m = 0; m < IMAGE_HEIGHT; m++) {
            for (int n = 0; n < IMAGE_WIDTH; n++) {
                image[m * IMAGE_WIDTH * 2 + n * 2] = raw_image.at<unsigned char>(m, n);
            }
        }
    }

    return image;
}
