
#include "GoalMatcher.h"
#include <stdio.h>
#include "GoalMatcherConstants.h"
//#include "AbsCoord.h"

GoalMatcher::GoalMatcher() {
    vocabLoaded = false;
    state       = STATE_READY;

    wasInitial = true;
    clearMap   = true;

    awayMapSize = 0;
    homeMapSize = 0;

    while ((int) obs.size() < WINDOW_SIZE) {
        obs.push_back(message::vision::Goal::Team::UNKNOWN_TEAM);
    }
}

// Load the visual dictionary for fast matching and starting map of features around the field
void GoalMatcher::loadVocab(std::string vocabFile) {
    tfidf.loadVocab(vocabFile);
    vocabLoaded = true;
}

void GoalMatcher::loadMap(std::string mapFile) {
    if (vocabLoaded) tfidf.loadMap(mapFile);
}

float GoalMatcher::getValidCosineScore() {
    return tfidf.getValidCosineScore();
}

int GoalMatcher::getValidInliers() {
    return tfidf.getValidInliers();
}

void GoalMatcher::setValidCosineScore(float x) {
    tfidf.setValidCosineScore(x);
}

void GoalMatcher::setValidInliers(int x) {
    tfidf.setValidInliers(x);
}

void GoalMatcher::printRANSACandSPAverages() {
    tfidf.printRANSACandBOVWandSPAverages();
}

int GoalMatcher::classifyGoalArea(std::shared_ptr<const message::vision::ClassifiedImage> frame,
                                  std::unique_ptr<std::vector<Ipoint>>& landmarks,
                                  std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                                  std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc,
                                  message::vision::Goal::Team& type,
                                  Eigen::MatrixXd* resultTable) {

    // if ((!vocabLoaded) || (!frame.wordMapped)) return 0; // Check valid data

    int num                                                    = 0;
    std::unique_ptr<std::priority_queue<MapEntry>> matchesPrim = std::make_unique<std::priority_queue<MapEntry>>();
    std::unique_ptr<std::priority_queue<MapEntry>> matchesSec  = std::make_unique<std::priority_queue<MapEntry>>();
    Eigen::VectorXf query;
    std::vector<std::vector<float>> query_pixLoc;
    unsigned int seed = 42;  // Not sure what this seed does, but it is supposed to come from the figure.
    type              = message::vision::Goal::Team::UNKNOWN_TEAM;

    // Augment landmarks with those from a previous matched frame if possible

    // if(frame.validSurf){
    // query = frame.landmark_tf_aug;
    // query_pixLoc = frame.landmark_pixLoc_aug;
    //} else {
    query        = *landmark_tf;
    query_pixLoc = *landmark_pixLoc;

    //}

    tfidf.searchDocument(query, query_pixLoc, matchesPrim, matchesSec, &seed, SEARCH_POSITIONS, resultTable);

    num = (int) matchesPrim->size();
    printf("Number of matchesPrim = %d\n", num);

    AbsCoord position;
    int i               = 0;
    int away_goal_votes = 0;
    int away_goal_count = 0;
    int home_goal_count = 0;
    while (!matchesPrim->empty() && i < SEARCH_POSITIONS) {
        i++;
        MapEntry entry = matchesPrim->top();
        matchesPrim->pop();
        position = entry.position;

        if (position.theta() < M_PI / 2 && position.theta() > -M_PI / 2) {
            away_goal_votes++;
            away_goal_count++;
        }
        else {
            away_goal_votes--;
            home_goal_count++;
        }
    }

    auto timer_s = std::chrono::system_clock::now();

    // Running Spatial Pyramid to attempt to recover some invalidated image matches
    if ((num <= MIN_CONSENSUS_DIFF)
        || ((away_goal_votes < MIN_CONSENSUS_DIFF) && (away_goal_votes > -MIN_CONSENSUS_DIFF))) {
        i = away_goal_count + home_goal_count;
        printf("RUNNING SPATIAL PYRAMID with i=%d\n", i);
        while (!matchesSec->empty() && i <= SEARCH_POSITIONS) {
            MapEntry entry = matchesSec->top();
            matchesSec->pop();
            position = entry.position;

            if (position.theta() < M_PI / 2 && position.theta() > -M_PI / 2) {
                away_goal_votes++;
                away_goal_count++;
                (*resultTable)(0, 4) = (*resultTable)(0, 4) + 1.0;
                num++;
            }
            else {
                away_goal_votes--;
                home_goal_count++;
                (*resultTable)(0, 5) = (*resultTable)(0, 5) + 1.0;
                num++;
            }
            i++;
        }
    }
    auto timer_e = std::chrono::system_clock::now();
    auto timer   = std::chrono::duration_cast<std::chrono::microseconds>(timer_e - timer_s);
    printf("Additional SP counting time: %0.0f\n", (float) timer.count());


    // Now look for a consensus position
    if ((away_goal_votes >= MIN_CONSENSUS_DIFF) && (num > MIN_CONSENSUS_DIFF)) {
        type = message::vision::Goal::Team::OPPONENT;
        printf("This is the Opponent's goal. Away/Home votes: %d/%d\n", away_goal_count, home_goal_count);
        return num;
    }
    else if ((away_goal_votes <= -MIN_CONSENSUS_DIFF) && (num > MIN_CONSENSUS_DIFF)) {
        type = message::vision::Goal::Team::OWN;
        printf("This is our own goal. Away/Home votes: %d/%d\n", away_goal_count, home_goal_count);
        return num;
    }

    return num;
}


void GoalMatcher::process(std::shared_ptr<const message::vision::ClassifiedImage> frame,
                          std::unique_ptr<std::vector<Ipoint>>& landmarks,
                          std::unique_ptr<Eigen::VectorXf>& landmark_tf,
                          std::unique_ptr<std::vector<std::vector<float>>>& landmark_pixLoc,
                          const message::localisation::Field& field,
                          float& awayGoalProb,
                          std::string mapFile,
                          Eigen::MatrixXd* resultTable) {

    // Adjust robotPos for head yaw
    AbsCoord position = {field.position[0], field.position[1], field.position[2]};
    printf("Robot is in position: <%0.1f,%0.1f,%0.1f>\n", position.x(), position.y(), position.theta());
    /*
    if (isnan(position.x()) || isnan(position.y()) || isnan(position.theta())){
        return; // Need to already be localised to one side of the field or the other
    }
    position.theta() = position.theta() + headYaw;
    */
    if (state == STATE_INITIAL) {
        wasInitial = true;
        clearMap   = true;
        printf("state was STATE_INITIAL\n");
    }
    else if (state != STATE_READY) {
        wasInitial = false;
        printf("state was not STATE_READY\n");
    }

    /****** TO DO ********/
    // Don't do anything if you are not really facing a field end

    if ((state == STATE_READY) && (wasInitial)) {
        // saving landmarks mode
        printf("Saving Landmarks mode:...\n");

        if (clearMap) {
            tfidf.clearMap();
            clearMap    = false;
            awayMapSize = 0;
            homeMapSize = 0;
            printf("map cleared\n");
        }

        // message::vision::Goal::Team type = message::vision::Goal::Team::UNKNOWN;
        // int num_matches = classifyGoalArea(frame, landmarks, landmark_tf, landmark_pixLoc, type);
        // if (num_matches < MIN_SAVE_MATCHES ){
        // printf("This appears to be a unique perspective of the goal\n");
        // Check which goal we are mapping
        bool awayGoal = false;
        // This only works if heading is w.r.t. the field and the x-axis is towards the opponent's goal.
        if (std::abs(field.position[2]) < M_PI * 0.5) {
            awayGoal = true;
            printf("We are looking at the OPPONENT goal\n");
        }
        else {
            printf("We are looking at the OWN goal\n");
        }

        Eigen::VectorXf tf_doc;
        std::vector<std::vector<float>> pixLoc;
        // Augment landmarks with those from a previous matched frame if possible
        // if(frame.validSurf){
        //  tf_doc = frame.landmark_tf_aug;
        //  pixLoc = frame.landmark_pixLoc_aug;
        //} else {
        tf_doc = *landmark_tf;
        pixLoc = *landmark_pixLoc;
        //}
        if (awayGoal && awayMapSize < MAP_MAX) {
            MapEntry document = MapEntry(position);
            tfidf.addDocumentToCorpus(document, tf_doc, pixLoc);
            vocabLoaded = true;
            tfidf.saveMap(mapFile);
            awayGoalProb = 0.9;  // Don't want to use 100% certainty, so 90% is a guess.
            printf("awayMapSize: %d->%d\tMapping away goal, position (%.1f, %.1f)\n",
                   awayMapSize,
                   awayMapSize + 1,
                   position.x(),
                   position.y());  // theta has been removed temporarily
            awayMapSize++;
        }
        else if (!awayGoal && homeMapSize < MAP_MAX) {
            MapEntry document = MapEntry(position);
            tfidf.addDocumentToCorpus(document, tf_doc, pixLoc);
            vocabLoaded = true;
            tfidf.saveMap(mapFile);
            awayGoalProb = 0.1;  // Don't want to use 100% certainty, so 10% is a guess.
            printf("homeMapSize: %d->%d\tMapping home goal, position (%.1f, %.1f)\n",
                   homeMapSize,
                   homeMapSize + 1,
                   position.x(),
                   position.y());  // theta has been removed temporarily
            homeMapSize++;
        }
        else {
            printf("I should map this view but the map is already full for awayGoal = ");
            printf(awayGoal ? "true\n" : "false\n");
        }
        //}
    }
    else {  // landmark retrieval and goal classification mode
        printf("Landmark retrieval mode\n");
        message::vision::Goal::Team type = message::vision::Goal::Team::UNKNOWN_TEAM;
        classifyGoalArea(frame, landmarks, landmark_tf, landmark_pixLoc, type, resultTable);
        // frame.goalArea = type;

        obs.insert(obs.begin(), type);
        obs.pop_back();
        float away = 1.f;
        float home = 1.f;
        for (int i = 0; i < (int) obs.size(); i++) {
            if (obs[i] == message::vision::Goal::Team::OWN) home += 1.f;
            if (obs[i] == message::vision::Goal::Team::OPPONENT) away += 1.f;
        }
        awayGoalProb = away / (home + away);
    }
}
