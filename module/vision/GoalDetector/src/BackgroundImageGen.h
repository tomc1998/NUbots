#include <iostream>
#include "GoalMatcherConstants.h"
#include "message/input/Image.h"
#include "message/localisation/Field.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "stdio.h"
#include "utility/vision/fourcc.h"

using namespace cv;
using message::input::Image;

Image backgroundImageGen(uint8_t imageNum, std::unique_ptr<message::localisation::Field>& field, double* d) {

    Mat raw_image;
    if (imageNum == 1) {
        *d                 = 53.0;  // Horizion height
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = 0;     // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image01.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 2) {
        *d                 = 15.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image02.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 3) {
        *d                 = 47.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image03.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 4) {
        *d                 = 52.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image04.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 5) {
        *d                 = 50.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image05.png", CV_LOAD_IMAGE_GRAYSCALE);
    }


    else if (imageNum == 6) {
        *d                 = 56.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image06.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 7) {
        *d                 = 56.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image07.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 8) {
        *d                 = 56.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image08.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 9) {
        *d                 = 53.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image09.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 10) {
        *d                 = 64.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image10.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 11) {
        *d                 = 64.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image11.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 12) {
        *d                 = 64.0;
        field->position[0] = 0;  // x position
        field->position[1] = 0;  // y position
        field->position[2] = 0;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image12.png", CV_LOAD_IMAGE_GRAYSCALE);
    }

    /*******************************************************************/
    /************************* OWN GOAL BACKGROUND IMAGES **************/
    /*******************************************************************/

    else if (imageNum == 13) {
        *d                 = 30.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back01.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 14) {
        *d                 = 30.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back02.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 15) {
        *d                 = 35.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back03.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 16) {
        *d                 = 37.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back04.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 17) {
        *d                 = 28.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back05.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 18) {
        *d                 = 22.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back06.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 19) {
        *d                 = 43.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back07.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 20) {
        *d                 = 30.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back08.png", CV_LOAD_IMAGE_GRAYSCALE);
    }
    else if (imageNum == 21) {
        *d                 = 39.0;
        field->position[0] = 0;     // x position
        field->position[1] = 0;     // y position
        field->position[2] = M_PI;  // x component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back09.png", CV_LOAD_IMAGE_GRAYSCALE);
    }


    // Converting matrix to a concatenated long vector
    std::vector<uint8_t> data(IMAGE_HEIGHT * IMAGE_WIDTH, 0);
    if (!raw_image.data) {
        std::cout << "Could not open or find the image" << std::endl;
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
