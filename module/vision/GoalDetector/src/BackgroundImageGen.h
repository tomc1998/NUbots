#include <iostream>
#include "GoalMatcherConstants.h"
#include "message/localisation/FieldObject.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;

std::vector<uint8_t> backgroundImageGen(uint8_t imageNum,
                                        std::unique_ptr<message::localisation::Self>& self,
                                        double* d) {
    std::vector<uint8_t> image(IMAGE_HEIGHT * IMAGE_WIDTH * 2, 0);
    Mat raw_image;
    if (imageNum == 1) {
        *d                = 53.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image01.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 2) {
        *d                = 15.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image02.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 3) {
        *d                = 47.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image03.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 4) {
        *d                = 52.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image04.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 5) {
        *d                = 50.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image05.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }


    else if (imageNum == 6) {
        *d                = 56.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image06.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 7) {
        *d                = 56.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image07.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 8) {
        *d                = 56.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image08.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 9) {
        *d                = 53.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image09.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 10) {
        *d                = 64.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image10.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 11) {
        *d                = 64.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image11.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 12) {
        *d                = 64.0;
        self->position[0] = 0;  // x position
        self->position[1] = 0;  // y position
        self->heading[0]  = 1;  // x component heading
        self->heading[1]  = 0;  // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/image12.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }

    /*******************************************************************/
    /************************* OWN GOAL BACKGROUND IMAGES **************/
    /*******************************************************************/

    else if (imageNum == 13) {
        *d                = 30.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back01.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 14) {
        *d                = 30.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back02.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 15) {
        *d                = 35.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back03.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 16) {
        *d                = 37.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back04.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 17) {
        *d                = 28.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back05.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 18) {
        *d                = 22.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back06.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 19) {
        *d                = 43.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back07.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 20) {
        *d                = 30.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back08.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }
    else if (imageNum == 21) {
        *d                = 39.0;
        self->position[0] = 0;   // x position
        self->position[1] = 0;   // y position
        self->heading[0]  = -1;  // x component heading
        self->heading[1]  = 0;   // y component heading
        raw_image = imread("/home/vagrant/NUbots/module/vision/GoalDetector/data/back09.png", CV_LOAD_IMAGE_GRAYSCALE);
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
    }


    return image;
}
