/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#include "MMKFRobotLocalisationEngine.h"
#include <chrono>
#include <algorithm>
#include "utility/time/time.h"
#include "utility/localisation/LocalisationFieldObject.h"
#include "messages/vision/VisionObjects.h"
#include "messages/localisation/FieldObject.h"

using utility::localisation::LFOId;
using utility::localisation::LocalisationFieldObject;
using utility::time::TimeDifferenceSeconds;
using messages::vision::VisionObject;
using messages::localisation::FakeOdometry;

namespace modules {
namespace localisation {

    void MMKFRobotLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time) {
        double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
        last_time_update_time_ = current_time;
        robot_models_.TimeUpdate(seconds);
    }

    void MMKFRobotLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time,
                                              const FakeOdometry& odom) {
        double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
        last_time_update_time_ = current_time;
        robot_models_.TimeUpdate(seconds, odom);
    }
    
    void MMKFRobotLocalisationEngine::TimeUpdate(std::chrono::system_clock::time_point current_time,
                                              const arma::mat44& odom) {
        double seconds = TimeDifferenceSeconds(current_time, last_time_update_time_);
        last_time_update_time_ = current_time;
        robot_models_.TimeUpdate(seconds, odom);
    }

    std::vector<LocalisationFieldObject> MMKFRobotLocalisationEngine::GetPossibleObjects(
            const messages::vision::Goal& ambiguous_object) {
        std::vector<LocalisationFieldObject> possible;

        if (ambiguous_object.type == messages::vision::Goal::Type::LEFT) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYL));
        }

        if (ambiguous_object.type == messages::vision::Goal::Type::RIGHT) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBR));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYR));
        }

        if (ambiguous_object.type == messages::vision::Goal::Type::UNKNOWN) {
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYL));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalBR));
            possible.push_back(field_description_->GetLFO(LFOId::kGoalYR));
        }

        return std::move(possible);
    }

    bool GoalPairObserved(
        const std::vector<messages::vision::Goal>& ambiguous_objects) {
        if (ambiguous_objects.size() != 2)
            return false;

        auto& oa = ambiguous_objects[0];
        auto& ob = ambiguous_objects[1];

        return
            (oa.type == messages::vision::Goal::Type::RIGHT &&
             ob.type == messages::vision::Goal::Type::LEFT) ||
            (oa.type == messages::vision::Goal::Type::LEFT &&
             ob.type == messages::vision::Goal::Type::RIGHT);
    }

    void MMKFRobotLocalisationEngine::ProcessAmbiguousObjects(
        const std::vector<messages::vision::Goal>& ambiguous_objects) {
        
        bool pair_observations_enabled = 
            cfg_.goal_pair_observation_enabled ||
            cfg_.angle_between_goals_observation_enabled;

        if (pair_observations_enabled && GoalPairObserved(ambiguous_objects)) {
            std::vector<messages::vision::VisionObject> vis_objs;
            // Ensure left goal is always first.
            if (ambiguous_objects[0].type == messages::vision::Goal::Type::LEFT){
                vis_objs = { ambiguous_objects[0], ambiguous_objects[1] };
            } else {
                vis_objs = { ambiguous_objects[1], ambiguous_objects[0] };
            }

            std::vector<std::vector<LocalisationFieldObject>> objs = {
                {field_description_->GetLFO(LFOId::kGoalBL),
                 field_description_->GetLFO(LFOId::kGoalBR)},
                {field_description_->GetLFO(LFOId::kGoalYL),
                 field_description_->GetLFO(LFOId::kGoalYR)}
            };

            if(cfg_.goal_pair_observation_enabled)
                robot_models_.AmbiguousMeasurementUpdate(vis_objs, objs);
            
            if (cfg_.angle_between_goals_observation_enabled)
                robot_models_.AmbiguousMultipleMeasurementUpdate(vis_objs, objs);
        } else {
            for (auto& ambiguous_object : ambiguous_objects) {
                // Get a vector of all field objects that the observed object could
                // possibly be
                auto possible_objects = GetPossibleObjects(ambiguous_object);
                robot_models_.AmbiguousMeasurementUpdate(ambiguous_object, possible_objects);
            }
        }

        robot_models_.PruneModels();
    }

    void MMKFRobotLocalisationEngine::IndividualStationaryObjectUpdate(
        const std::vector<messages::vision::Goal>& goals,
        float time_increment) {

        for (auto& observed_object : goals) {

            LocalisationFieldObject actual_object;

            if (observed_object.type == messages::vision::Goal::Type::LEFT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBL);

            if (observed_object.type == messages::vision::Goal::Type::RIGHT)
                actual_object = field_description_->GetLFO(LFOId::kGoalBR);

            robot_models_.MeasurementUpdate(observed_object, actual_object);
        }

        robot_models_.NormaliseAlphas();
    }

    /*! @brief Process objects
        Processes the field objects and perfroms the correction updates required from the observations.

        @param fobs The object information output by the vision module. This contains objects identified and their relative positions.
        @param time_increment The time that has elapsed since the previous localisation frame.
     */
    void MMKFRobotLocalisationEngine::ProcessObjects(const std::vector<messages::vision::Goal>& goals) {
        ProcessAmbiguousObjects(goals);
    }
}
}