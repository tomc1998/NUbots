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

#include "SoccerSimulator.h"
#include <nuclear>
#include <sstream>
#include "utility/math/angle.h"
#include "utility/math/coordinates.h"
#include "utility/nubugger/NUhelpers.h"
#include "utility/localisation/transform.h"
#include "utility/motion/ForwardKinematics.h"
#include "messages/vision/VisionObjects.h"
#include "messages/support/Configuration.h"
#include "messages/localisation/FieldObject.h"
#include "messages/input/ServoID.h"
#include "messages/motion/WalkCommand.h"
#include "messages/input/GameEvents/gameevents.h"


namespace modules {
namespace support {

    using messages::platform::darwin::ButtonMiddleDown;
    using messages::input::Sensors;
    using messages::input::ServoID;
    using utility::nubugger::drawArrow;
    using utility::nubugger::drawSphere;
    using utility::math::angle::normalizeAngle;
    using utility::math::angle::vectorToBearing;
    using utility::math::angle::bearingToUnitVector;
    using utility::math::coordinates::cartesianToSpherical;
    using utility::motion::kinematics::calculateRobotToIMU;
    using utility::localisation::transform::SphericalRobotObservation;
    using utility::localisation::transform::WorldToRobotTransform;
    using utility::localisation::transform::RobotToWorldTransform;
    using utility::nubugger::graph;
    using messages::support::Configuration;
    using messages::support::FieldDescription;
    using messages::motion::WalkCommand;
    using messages::motion::KickCommand;
    using messages::motion::KickFinished;
    using messages::motion::KickPlannerConfig;
    using messages::platform::darwin::DarwinSensors;
    using utility::math::matrix::Transform2D;
    using messages::support::Configuration;
    using messages::support::GlobalConfig;
    using namespace messages::input::gameevents;
    using utility::support::Expression;

    double triangle_wave(double t, double period) {
        auto a = period; // / 2.0;
        auto k = t / a;
        return 2.0 * std::abs(2.0 * (k - std::floor(k + 0.5))) - 1.0;
    }
    double sawtooth_wave(double t, double period) {
        return 2.0 * std::fmod(t / period, 1.0) - 1.0;
    }
    double square_wave(double t, double period) {
        return std::copysign(1.0, sawtooth_wave(t, period));
    }
    double sine_wave(double t, double period) {
        return std::sin((2.0 * M_PI * t) / period);
    }
    double SoccerSimulator::absolute_time() {
        auto now = NUClear::clock::now();
        auto msSinceStart = std::chrono::duration_cast<std::chrono::microseconds>(now - moduleStartupTime).count();
        double ms = static_cast<double>(msSinceStart);
        double t = ms * 1e-6;
        return t;
    }

    void SoccerSimulator::updateConfiguration(const Configuration<SoccerSimulatorConfig>& config, const GlobalConfig& globalConfig) {

        moduleStartupTime = NUClear::clock::now();

        cfg_.simulate_goal_observations = config["vision"]["goal_observations"].as<bool>();
        cfg_.simulate_ball_observations = config["vision"]["ball_observations"].as<bool>();
        cfg_.distinguish_own_and_opponent_goals = config["vision"]["distinguish_own_and_opponent_goals"].as<bool>();

        cfg_.robot.motion_type = motionTypeFromString(config["robot"]["motion_type"].as<std::string>());
        cfg_.robot.path.period = config["robot"]["path"]["period"].as<Expression>();
        cfg_.robot.path.x_amp = config["robot"]["path"]["x_amp"].as<Expression>();
        cfg_.robot.path.y_amp = config["robot"]["path"]["y_amp"].as<Expression>();
        cfg_.robot.path.type = pathTypeFromString(config["robot"]["path"]["type"].as<std::string>());

        cfg_.ball.motion_type = motionTypeFromString(config["ball"]["motion_type"].as<std::string>());
        cfg_.ball.path.period = config["ball"]["path"]["period"].as<Expression>();
        cfg_.ball.path.x_amp = config["ball"]["path"]["x_amp"].as<Expression>();
        cfg_.ball.path.y_amp = config["ball"]["path"]["y_amp"].as<Expression>();
        cfg_.ball.path.type = pathTypeFromString(config["ball"]["path"]["type"].as<std::string>());

        world.robotPose = config["initial"]["robot_pose"].as<arma::vec3>();
        world.ball.position = config["initial"]["ball"]["position"].as<arma::vec3>();
        world.ball.diameter = config["initial"]["ball"]["diameter"].as<Expression>();

        cfg_.blind_robot = config["blind_robot"].as<bool>();

        cfg_.vision_error(0) = config["vision"]["variance"]["r"]["proportional_factor"].as<Expression>();
        cfg_.vision_error(1) = config["vision"]["variance"]["r"]["min_error"].as<Expression>();
        cfg_.vision_error(2) = config["vision"]["variance"]["theta"].as<Expression>();
        cfg_.vision_error(3) = config["vision"]["variance"]["phi"].as<Expression>();

        kicking = false;
        PLAYER_ID = globalConfig.playerId;

        cfg_.auto_start_behaviour = config["auto_start_behaviour"].as<bool>();
    }

    SoccerSimulator::SoccerSimulator(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)) {

        on<Trigger<FieldDescription>>("FieldDescription Update", [this](const FieldDescription& desc) {

            field_description_ = std::make_shared<FieldDescription>(desc);

            goalPosts.clear();

            arma::vec3 goal_opp_r = {field_description_->goalpost_opp_r[0],field_description_->goalpost_opp_r[1],0};
            goalPosts.push_back(VirtualGoalPost(goal_opp_r, 1.1, Goal::Side::RIGHT, Goal::Team::OPPONENT));

            arma::vec3 goal_opp_l = {field_description_->goalpost_opp_l[0],field_description_->goalpost_opp_l[1],0};
            goalPosts.push_back(VirtualGoalPost(goal_opp_l, 1.1, Goal::Side::LEFT, Goal::Team::OPPONENT));

            arma::vec3 goal_own_r = {field_description_->goalpost_own_r[0],field_description_->goalpost_own_r[1],0};
            goalPosts.push_back(VirtualGoalPost(goal_own_r, 1.1, Goal::Side::RIGHT, Goal::Team::OWN));

            arma::vec3 goal_own_l = {field_description_->goalpost_own_l[0],field_description_->goalpost_own_l[1],0};
            goalPosts.push_back(VirtualGoalPost(goal_own_l, 1.1, Goal::Side::LEFT, Goal::Team::OWN));

            for(auto& g : goalPosts){
                log("goalPost", g.position.t());
            }

        });

        on<With<Configuration<SoccerSimulatorConfig>>, Trigger<GlobalConfig>>("Soccer Simulator Configuration", std::bind(std::mem_fn(&SoccerSimulator::updateConfiguration), this, std::placeholders::_1, std::placeholders::_2));
        on<Trigger<Configuration<SoccerSimulatorConfig>>, With<GlobalConfig>>("Soccer Simulator Configuration", std::bind(std::mem_fn(&SoccerSimulator::updateConfiguration), this, std::placeholders::_1, std::placeholders::_2));

        on<Trigger<KickPlannerConfig>>("Get Kick Planner Config", [this](const KickPlannerConfig& cfg){
            kick_cfg = cfg;
        });

        on<Trigger<KickCommand>>("Simulator Queue KickCommand",[this](const KickCommand& k){
            kickQueue.push(k);
            kicking = true;
        });
        on<Trigger<KickFinished>>("Simulator Kick Finished",[this](const KickFinished&){
            kicking = false;
        });

        on<
            Trigger<Every<SIMULATION_UPDATE_FREQUENCY, Per<std::chrono::seconds>>>,
            With<Optional<WalkCommand>>
        >("Robot motion", [this](const time_t&,
                                 const std::shared_ptr<const WalkCommand>& walkCommand) {

            Transform2D diff;

            switch (cfg_.robot.motion_type){
                case MotionType::NONE:
                    world.robotVelocity = Transform2D({ 0, 0 ,0 });
                    break;

                case MotionType::PATH:

                    world.robotPose.xy() = getPath(cfg_.robot.path);

                    diff = world.robotPose - oldRobotPose;
                    //Face along direction of movement
                    world.robotPose.angle() = vectorToBearing(diff.xy());

                    world.robotVelocity = Transform2D({arma::norm(diff) * SIMULATION_UPDATE_FREQUENCY, 0, 0}); //Robot coordinates
                    break;

                case MotionType::MOTION:
                //Update based on walk engine
                    if(walkCommand && !kicking) {
                        world.robotVelocity = walkCommand->command;
                    } else {
                        world.robotVelocity = utility::math::matrix::Transform2D({0,0,0});
                    }
                    world.robotVelocity.xy() = world.robotPose.rotation() * world.robotVelocity.xy();
                    world.robotPose += world.robotVelocity / SIMULATION_UPDATE_FREQUENCY;
                    break;
            }
            // Update ball position
            switch (cfg_.ball.motion_type) {
                case MotionType::NONE:
                    world.ball.velocity = { 0, 0 , 0};
                    break;

                case MotionType::PATH:

                    world.ball.position.rows(0,1) = getPath(cfg_.ball.path);

                    diff = world.ball.position - oldBallPose;

                    world.ball.velocity = Transform2D({arma::norm(diff) * SIMULATION_UPDATE_FREQUENCY, 0, 0}); //Robot coordinates
                    break;

                case MotionType::MOTION:
                    if(!kickQueue.empty()){
                        //Get last queue
                        KickCommand lastKickCommand = kickQueue.back();
                        //Empty queue
                        std::queue<KickCommand>().swap(kickQueue);
                        //Check if kick worked:
                        Transform2D relativeBallPose = world.robotPose.worldToLocal(world.ball.position);

                        if( relativeBallPose.x() < kick_cfg.MAX_BALL_DISTANCE &&
                            std::fabs(relativeBallPose.y()) < kick_cfg.KICK_CORRIDOR_WIDTH / 2){
                                world.ball.position.rows(0,1) += world.robotPose.rotation() * lastKickCommand.direction.rows(0, 1);
                        }
                    }
                    break;
            }

            // Emit the change in orientation as a DarwinSensors::Gyroscope,
            // to be handled by HardwareSimulator.
            emit(computeGyro(world.robotPose.angle(), oldRobotPose.angle()));

            oldRobotPose = world.robotPose;
            oldBallPose = world.ball.position;
        });

        // Simulate Vision
        on<Trigger<Every<30, Per<std::chrono::seconds>>>,
            With<Raw<Sensors>>,
            With<CameraParameters>,
            Options<Sync<SoccerSimulator>>
            >("Vision Simulation", [this](const time_t&,
                const std::shared_ptr<Sensors>& sensors,
                const CameraParameters& camParams) {

            if (field_description_ == nullptr) {
                NUClear::log(__FILE__, __LINE__, ": field_description_ == nullptr");
                return;
            }

            if (cfg_.simulate_goal_observations) {
                auto goals = std::make_unique<std::vector<messages::vision::Goal>>();
                if (cfg_.blind_robot) {
                    emit(std::move(goals));
                    return;
                }

                // for (auto& g : goalPosts) {
                for (auto& g : goalPosts) {

                    // Detect the goal:
                    auto m = g.detect(camParams, world.robotPose, sensors, cfg_.vision_error);

                    if (!m.measurements.empty()) {
                        if (!cfg_.distinguish_own_and_opponent_goals) {
                            m.team = messages::vision::Goal::Team::UNKNOWN;
                        }
                        goals->push_back(m);
                    }
                }

                emit(std::move(goals));

            } else {
                // Emit current self exactly
                auto r = std::make_unique<std::vector<messages::localisation::Self>>();
                r->push_back(messages::localisation::Self());
                r->back().position = world.robotPose.xy();
                r->back().heading = bearingToUnitVector(world.robotPose.angle());
                r->back().velocity = world.robotVelocity.rows(0,1);
                r->back().position_cov = 0.00001 * arma::eye(2,2);
                r->back().last_measurement_time = NUClear::clock::now();
                emit(std::move(r));
            }


            if (cfg_.simulate_ball_observations) {
                auto ball_vec = std::make_unique<std::vector<messages::vision::Ball>>();
                if (cfg_.blind_robot) {
                    emit(std::move(ball_vec));
                    return;
                }

                auto ball = world.ball.detect(camParams, world.robotPose, sensors, cfg_.vision_error);

                if (!ball.measurements.empty()) {

                    ball_vec->push_back(ball);

                }

                emit(std::move(ball_vec));

            } else {
                // Emit current ball exactly
                auto b = std::make_unique<messages::localisation::Ball>();
                b->position = world.robotPose.worldToLocal(world.ball.position).xy();
                b->velocity = world.robotPose.rotation().t() * world.ball.velocity.rows(0,1);
                b->position_cov = 0.00001 * arma::eye(2,2);
                b->last_measurement_time = NUClear::clock::now();
                emit(std::make_unique<std::vector<messages::localisation::Ball>>(
                        std::vector<messages::localisation::Ball>(1,*b)
                    ));
                emit(std::move(b));
            }

        });

        // Emit exact position to NUbugger
        on<Trigger<Every<100, std::chrono::milliseconds>>>(
        "Emit True Robot Position",
            [this](const time_t&) {

            arma::vec2 bearingVector = world.robotPose.rotation() * arma::vec2({1,0});
            arma::vec3 robotHeadingVector = {bearingVector[0], bearingVector[1], 0};
            emit(drawArrow("robot", {world.robotPose.x(), world.robotPose.y(), 0}, robotHeadingVector, 1, 0));

            emit(drawSphere("ball", {world.ball.position(0), world.ball.position(1), 0}, 0.1, 0));
        });

        on<Trigger<Startup>>("SoccerSimulator Startup",[this](const Startup&){
            if (cfg_.auto_start_behaviour) {
                auto time = NUClear::clock::now();
                emit(std::make_unique<Unpenalisation<SELF>>(Unpenalisation<SELF>{PLAYER_ID}));
                emit(std::make_unique<GamePhase<Phase::PLAYING>>(GamePhase<Phase::PLAYING>{time, time}));
                emit(std::make_unique<Phase>(Phase::PLAYING));
            }

        });
    }

    std::unique_ptr<DarwinSensors::Gyroscope> SoccerSimulator::computeGyro(float heading, float oldHeading){
        // float dHeading = utility::math::angle::difference(heading, oldHeading);
        float dHeading = heading - oldHeading;

        auto g = std::make_unique<DarwinSensors::Gyroscope>();
        g->x = 0;
        g->y = 0;
        g->z = dHeading;
        return std::move(g);
    }

    arma::vec2 SoccerSimulator::getPath(SoccerSimulator::Config::Motion::Path p){
        auto t = absolute_time();
        float wave1,wave2;
        switch(p.type){
            case PathType::SIN:
                wave1 = p.x_amp * sine_wave(t, p.period);
                wave2 = p.y_amp * sine_wave(t + (p.period / 4.0), p.period);
                break;
            case PathType::TRIANGLE:
                wave1 = p.x_amp * triangle_wave(t, p.period);
                wave2 = p.y_amp * triangle_wave(t + (p.period / 4.0), p.period);
                break;
            default:
                std::stringstream str;
                str << __FILE__ << ", " << __LINE__ << ": " << __func__ << ": unknown p.type.";
                throw std::runtime_error(str.str());
        }
        return arma::vec2({wave1,wave2});
    }

}
}

