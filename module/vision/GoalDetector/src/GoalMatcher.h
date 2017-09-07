//#ifndef MODULES_VISION_GOALMATCHER_H
//#define MODULES_VISION_GOALMATCHER_H

#include <nuclear>
#include <queue>
#include <string>

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include "GoalMatcherConstants.h"
#include "Ipoint.h"
#include "Tfidf.h"
#include "message/input/Image.h"
#include "message/localisation/Field.h"
#include "message/vision/ClassifiedImage.h"
#include "message/vision/VisionObjects.h"
#define SEARCH_POSITIONS 8    // how many of the top responses to look at
#define MIN_CONSENSUS_DIFF 2  // eg. 4 against 2 if search positions == 6
#define MIN_SAVE_MATCHES 4    // How many hits you need to stop you saving the image again
#define MAP_MAX 40            // maximum number of images to map for each goal
#define WINDOW_SIZE 10        // how big our rolling window of observations is, not so big that it could possibly
// include both goal areas even at maximum robot turn and head turn combined

class GoalMatcher {
public:
    GoalMatcher();

    // Load a saved visual dictionary for fast matching and starting map of features around the field
    void loadVocab(std::string vocabFile);
    void loadMap(std::string mapFile);
    float getValidCosineScore();
    int getValidInliers();
    void setValidCosineScore(float x);
    void setValidInliers(int x);
    void printRANSACandSPAverages();

    // Tries to classify which end the visible goals are at, based on background landmarks, or learns
    // landmarks if in first ready state for the half
    void process(std::shared_ptr<const message::vision::ClassifiedImage> frame,
                 std::unique_ptr<std::vector<Ipoint>>& landmarks,
                 std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                 std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc,
                 const message::localisation::Field& field,
                 float& awayGoalProb,
                 std::string mapFile,
                 Eigen::MatrixXd* resultTable);

    void setWasInitial(bool x) {
        wasInitial = x;
    }

    uint8_t state;
    int awayMapSize;
    int homeMapSize;

private:
    // Tries to classify which field end we are facing, based on background landmarks, returns the number of matches
    int classifyGoalArea(std::shared_ptr<const message::vision::ClassifiedImage> frame,
                         std::unique_ptr<std::vector<Ipoint>>& landmarks,
                         std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                         std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc,
                         message::vision::Goal::Team& type,
                         Eigen::MatrixXd* resultTable);

    Tfidf tfidf;

    bool vocabLoaded;
    bool wasInitial;
    bool clearMap;
    std::vector<message::vision::Goal::Team> obs;
};
//#endif
