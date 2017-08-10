#pragma once

#include <math.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <limits>
#include <vector>
#include "GoalMatcherConstants.h"

struct Ipoint {

    //! Constructor
    Ipoint(){};

    //! Destructor
    // virtual ~Ipoint() {};

    //! Gets the distance in descriptor space between Ipoints
    float operator-(const Ipoint& rhs) {
        if (this->laplacian - rhs.laplacian != 0) return std::numeric_limits<float>::max();  // can't be match
        float sum = 0.f;
        for (int i = 0; i < SURF_DESCRIPTOR_LENGTH; ++i) {
            sum += (this->descriptor[i] - rhs.descriptor[i]) * (this->descriptor[i] - rhs.descriptor[i]);
        }
        return sqrt(sum);
    };

    //! Coordinates of the detected interest point
    float x, y;

    //! Detected scale
    float scale;

    //! Sign of laplacian for fast matching purposes
    int laplacian;

    //! If we think this point is on a robot
    int isRobot;

    //! Vector of descriptor components
    float descriptor[SURF_DESCRIPTOR_LENGTH];

    template <class Archive>
    void serialize(Archive& ar, const unsigned int file_version) {
        ar& x& y& scale;
        ar& laplacian& isRobot;
        ar& descriptor;
    }
};
