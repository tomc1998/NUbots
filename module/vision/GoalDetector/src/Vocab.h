#pragma once

#include <stdio.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <string>
#include <vector>
#include "Cluster.h"

#include "GoalMatcherConstants.h"
#include "Ipoint.h"

class Vocab {

public:
    //! Constructor
    Vocab() {
        vec_length = 0;
    };

    int getSize() {
        return vec_length;
    }

    //! Learns a set of visual words from larger set of interest points
    void learn(std::vector<Ipoint> ipts, int num_words);

    //! Loads a set of visual words for use
    void loadVocabFile(std::string filename);

    //! Map a set of interest points to a sparse term frequency vector while preserving
    // the pixel locations of the interest points
    Eigen::VectorXf mapToVec(std::unique_ptr<std::vector<Ipoint>>& ipts,
                             std::unique_ptr<std::vector<std::vector<float>>>& pixel_location);


private:
    int vec_length;

    std::vector<Ipoint> pos_words;
    std::vector<Ipoint> neg_words;

    std::vector<Ipoint> pos_stop_words;
    std::vector<Ipoint> neg_stop_words;
};
