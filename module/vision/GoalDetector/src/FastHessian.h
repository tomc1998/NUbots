#pragma once

#include <vector>
#include "GoalMatcherConstants.h"
#include "Integral.h"
#include "Ipoint.h"
#include "Responselayer.h"

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_RESET "\x1b[0m"


class FastHessian {

public:
    //! Constructor with image and y_coords for 1D
    FastHessian(std::unique_ptr<std::vector<float>>& img, std::unique_ptr<std::vector<Ipoint>>& ipts);

    //! Find the image features and write into vector of features
    void getIpoints();

private:
    //---------------- Private Functions -----------------//
    //! Build map of DoH responses
    void buildResponseMap();

    //! Calculate DoH responses for supplied Horizon layer
    void buildHorizonResponseLayer(ResponseLayer* rl);

    //! 3x3 Extrema test
    inline int isExtremum(int c,
                          std::vector<ResponseLayer>::iterator t,
                          std::vector<ResponseLayer>::iterator m,
                          std::vector<ResponseLayer>::iterator b);

    //! Save function
    inline void saveExtremum(int c,
                             std::vector<ResponseLayer>::iterator t,
                             std::vector<ResponseLayer>::iterator m,
                             std::vector<ResponseLayer>::iterator b);

    //---------------- Private Variables -----------------//

    //! Reference to vector of features passed from outside
    std::unique_ptr<std::vector<Ipoint>>& ipts;

    //! Pointer to the integral Image, and its attributes
    std::unique_ptr<std::vector<float>>& img;

    //! Response stack of determinant of hessian values
    std::vector<ResponseLayer> responseMap;
};
