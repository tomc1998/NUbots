
#include "SurfDetection.h"

#include "FastHessian.h"
#include "Integral.h"
#include "Ipoint.h"

#include <stdio.h>
#include <armadillo>
#include "message/input/Image.h"
#include "message/vision/ClassifiedImage.h"
#include "utility/math/geometry/Line.h"
#include "utility/vision/ClassifiedImage.h"

namespace module {
namespace vision {
    using message::input::Image;
    using message::vision::ClassifiedImage;
    using utility::math::geometry::Line;

    SurfDetection::SurfDetection(std::shared_ptr<const message::vision::ClassifiedImage> frame_2) : frame_p(frame_2) {}


    // Loads vocab ready for use, returns the size of the vocab in use
    int SurfDetection::loadVocab(std::string vocabFile) {

        vocab.loadVocabFile(vocabFile);
        int T = vocab.getSize();
        printf("Loaded vocab of %d words for SurfDetection\n", T);
        return T;
    }

    bool SurfDetection::vocabLoaded() {

        if (vocab.getSize() == 0)
            return false;
        else
            return true;
    }


    void SurfDetection::findLandmarks(std::unique_ptr<std::vector<Ipoint>>& landmarks_out,
                                      std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                                      std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc) {

        // Get horizon location
        uint w, h;
        w                 = frame_p->dimensions[0];
        h                 = frame_p->dimensions[1];
        int left_horizon  = utility::vision::visualHorizonAtPoint(*frame_p, 0);
        int right_horizon = utility::vision::visualHorizonAtPoint(*frame_p, w - 1);

        // Check horizon is within frame
        if (left_horizon >= 0 && left_horizon < h && right_horizon >= 0 && right_horizon < h) {

            // Create integral-image representation of the image

            Integral(int_img, frame_p, left_horizon, right_horizon);

            // For testing purposes...needs to be deleted
            // for (int i=0;i<IMAGE_WIDTH;++i){
            //  printf("%0.1f ",int_img->at(i));
            //}
            // printf("\n");

            // Create Fast Hessian Object

            FastHessian fh(int_img, landmarks);
            // Extract interest points and store in vector ipts
            fh.getIpoints();
        }

        int totaln = landmarks->size();
        printf("SURF landmarks found: %d\n", totaln);
        // for (int count = 0; count < totaln; count++){
        //  printf("Ipoint %d: (%0.1f,%0.1f) scale:
        //  %0.1f\n",count+1,landmarks->at(count).x,landmarks->at(count).y,landmarks->at(count).scale);
        //}

        // Check there are Ipoints to be described
        if (totaln > 0) {
            // Extract the descriptors for the ipts
            getHorizonDescriptors();
        }


        // If no vocab loaded, it just finds the raw interest points
        if (vocab.getSize() != 0) {
            // Map the interest points to visual words, if no landmarks it will correctly initialise
            *landmark_tf = vocab.mapToVec(landmarks, landmark_pixLoc);
            wordMapped   = true;
        }

        // return landmark
        landmarks_out = std::move(landmarks);
    }


    //! Describe all features in the supplied vector
    void SurfDetection::getHorizonDescriptors() {

        // Get the size of the vector for fixed loop bounds
        int ipts_size = (int) landmarks->size();

        for (int i = 0; i < ipts_size; ++i) {
            // Set the Ipoint to be described
            index = i;
            getHorizonDescriptor();
        }
    }


    //-------------------------------------------------------

    //! Our modified descriptor.
    inline void SurfDetection::getHorizonDescriptor() {
        int x, sample_x, count = 0;
        float scale, *desc, dx, mdx;
        float rx = 0.f, len = 0.f;

        std::vector<Ipoint>::iterator ipt =
            landmarks->begin() + index;  // replaces: Ipoint *ipt = &frame_p->landmarks[index];

        scale = ipt->scale;

        x    = fRound(ipt->x);
        desc = ipt->descriptor;

        for (int subregion = -1; subregion < 2; subregion++) {
            dx = mdx = 0.f;
            for (int sample = 0; sample < SURF_DESCRIPTOR_SAMPLES; sample++) {
                sample_x = fRound(x + (subregion * SURF_DESCRIPTOR_SAMPLES * scale
                                       + (sample - (SURF_DESCRIPTOR_SAMPLES / 2)) * scale));

                // Get the haar wavelet responses
                rx = haar(sample_x, 2 * fRound(scale));
                dx += rx;
                mdx += fabs(rx);
            }
            desc[count++] = dx;
            desc[count++] = mdx;
            len += (dx * dx + mdx * mdx);
        }

        // Convert to Unit Vector
        len = sqrt(len);
        for (int i = 0; i < SURF_DESCRIPTOR_LENGTH; ++i) {
            desc[i] /= len;
        }
    }

    //-------------------------------------------------------


    //! Calculate Haar wavelet responses in x direction (1D only)
    inline float SurfDetection::haar(int column, int s) {
        return BoxIntegral(int_img, column, s / 2) - 1.f * BoxIntegral(int_img, column - s / 2, s / 2);
    }

    //-------------------------------------------------------
}
}
