#ifndef MODULES_VISION_SURFDETECTION_H
#define MODULES_VISION_SURFDETECTION_H

#include <nuclear>

#include <eigen3/Eigen/Core>
#include "GoalMatcherConstants.h"
#include "Ipoint.h"
#include "Vocab.h"
#include "message/vision/ClassifiedImage.h"

namespace module {
namespace vision {


    class SurfDetection {
    public:
        SurfDetection(std::shared_ptr<const message::vision::ClassifiedImage<message::vision::ObjectClass>> frame_2);

        // find interest points
        void findLandmarks(std::unique_ptr<std::vector<Ipoint>>& landmarks_out,
                           std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                           std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc);

        //! Loads vocab ready for use, returns the size of the vocab
        int loadVocab(std::string vocabFile);

        bool isWordMapped() {
            return wordMapped;
        }

        // is a vocab loaded
        bool vocabLoaded();


    private:
        //---------------- Private Functions -----------------//

        //! Get all descriptors
        void getHorizonDescriptors();

        //! Our modified descriptor
        void getHorizonDescriptor();

        //! Calculate Haar wavelet response
        inline float haar(int column, int size);

        //! Round float to nearest integer

        inline int fRound(float flt) {
            return (int) floor(flt + 0.5f);
        }


        //---------------- Private Variables -----------------//

        // the Vocab used to map features to visual words
        Vocab vocab;

        bool wordMapped = false;  // if landmarks are mapped to visual words

        // Landmarks pointer
        std::unique_ptr<std::vector<Ipoint>> landmarks = std::make_unique<std::vector<Ipoint>>();

        // the Classified image structure
        std::shared_ptr<const message::vision::ClassifiedImage<message::vision::ObjectClass>> frame_p;

        // the integral horizon image
        std::unique_ptr<std::vector<float>> int_img =
            std::make_unique<std::vector<float>>(IMAGE_WIDTH / SURF_SUBSAMPLE, 0.0);

        //! Index of current Ipoint in the vector
        int index;
    };
}
}
#endif
