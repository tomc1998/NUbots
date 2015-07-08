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

#ifndef MODULES_MOTION_IKKICKCONTROLLERS_H
#define MODULES_MOTION_IKKICKCONTROLLERS_H

#include "utility/math/matrix/Transform3D.h"
#include "messages/input/Sensors.h"
#include "messages/input/LimbID.h"
#include "messages/support/Configuration.h"
#include "messages/input/ServoID.h"
#include <armadillo>
#include <nuclear>

namespace modules{
namespace motion{

		enum MotionStage {
			READY = 0,
			RUNNING = 1,
			STOPPING = 2,
			FINISHED = 3
		};

        struct IKKickConfig{
			static constexpr const char* CONFIGURATION_PATH = "IKKick.yaml";
        };

		class SixDOFFrame {
			// enum InterpolationType {
			// 	LINEAR = 0,
			// 	SERVO = 1
			//TODO:
			// };
			// InterpolationType interpolation = LINEAR;
		public:
			utility::math::matrix::Transform3D pose;
			float duration;
			SixDOFFrame():pose(){}
			SixDOFFrame(utility::math::matrix::Transform3D pose_, float duration_) : pose(pose_), duration(duration_){}
			SixDOFFrame(const YAML::Node& config){
				duration = config["duration"].as<float>();
				arma::vec3 pos = config["pos"].as<arma::vec>();
				arma::vec3 orientation = (180.0 / M_PI ) * config["orientation"].as<arma::vec>();
				pose = utility::math::matrix::Transform3D();
				pose.rotateX(orientation[0]);
				pose.rotateY(orientation[1]);
				pose.rotateZ(orientation[2]);
				pose.translation() = pos;
			};
			//TODO:
			// std::map<messages::input::ServoID, float> jointGains;
		};	

		class Animator{
		public:
			std::vector<SixDOFFrame> frames;
			int i = 0;
			Animator(){frames.push_back(SixDOFFrame{utility::math::matrix::Transform3D(),0});}
			Animator(std::vector<SixDOFFrame> frames_){
				frames = frames_;
			}
			int clampPrev(int k) const {return std::max(std::min(k,int(frames.size()-2)),0);}
			int clampCurrent(int k) const {return std::max(std::min(k,int(frames.size()-1)),0);}
			void next(){i = clampPrev(i+1);}
			void reset(){i = 0;}
			const SixDOFFrame& currentFrame() const {return frames[clampCurrent(i+1)];}
			const SixDOFFrame& previousFrame() const {return frames[i];}
			const SixDOFFrame& startFrame() const {return frames[0];}
			bool stable(){return i >= int(frames.size()-2);}
		};



		class SixDOFFootController{
			protected:
				MotionStage stage = MotionStage::READY;
				bool stable = false;

				//State variables
				messages::input::LimbID supportFoot;

				float forward_duration;
				float return_duration;
				Animator anim;

				arma::vec3 ballPosition;
				arma::vec3 goalPosition;

				NUClear::clock::time_point motionStartTime;
			public:
				
				virtual void computeStartMotion(const messages::input::Sensors& sensors) = 0;
				virtual void computeStopMotion(const messages::input::Sensors& sensors) = 0;

				void start(const messages::input::Sensors& sensors){
					if(stage == MotionStage::READY){
        				anim.reset();
						stage = MotionStage::RUNNING;
						stable = false;
        				motionStartTime = sensors.timestamp;
						computeStartMotion(sensors);
					}
				}

				void stop(const messages::input::Sensors& sensors){
					if(stage == MotionStage::RUNNING){
        				anim.reset();
						stage = MotionStage::STOPPING;
						stable = false;
						motionStartTime = sensors.timestamp;
						computeStopMotion(sensors);
					}
				}

				bool isRunning()	{return stage == MotionStage::RUNNING || stage == MotionStage::STOPPING;}
				bool isStable()		{return stable;}
				bool isFinished()   {return stage == MotionStage::FINISHED;}
				void reset()		{stage = MotionStage::READY; stable = false; anim.reset();}


				void setKickParameters(messages::input::LimbID supportFoot_, arma::vec3 ballPosition_, arma::vec3 goalPosition_) {
					supportFoot = supportFoot_;
					ballPosition = ballPosition_;
					goalPosition = goalPosition_;
					reset();
				}

				utility::math::matrix::Transform3D getTorsoPose(const messages::input::Sensors& sensors) {
			        // Get our foot positions
			        auto leftFoot = sensors.forwardKinematics.find(messages::input::ServoID::L_ANKLE_ROLL)->second;
			        auto rightFoot = sensors.forwardKinematics.find(messages::input::ServoID::R_ANKLE_ROLL)->second;

			        // Find position vector from support foot to torso in support foot coordinates.
		        	return supportFoot == messages::input::LimbID::LEFT_LEG ? leftFoot.i() : rightFoot.i();
		        }

				utility::math::matrix::Transform3D getFootPose(const messages::input::Sensors& sensors) {
					auto result = utility::math::matrix::Transform3D();
					if(stage == MotionStage::RUNNING || stage == MotionStage::STOPPING) {
			
						double elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(sensors.timestamp - motionStartTime).count() * 1e-6;
						float alpha = (anim.currentFrame().duration != 0)
										?	std::fmax(0,std::fmin(elapsedTime / anim.currentFrame().duration, 1))
										:	1;
						result = utility::math::matrix::Transform3D::interpolate(anim.previousFrame().pose,anim.currentFrame().pose,alpha);
						if(alpha >= 1){
							stable = anim.stable();
							if(!stable){
								anim.next();
								motionStartTime = sensors.timestamp;
							}
						}
						if(stable && stage == MotionStage::STOPPING){
							stage = MotionStage::FINISHED;
						}
		        	}
		        	return result;
				}

				virtual void configure(const messages::support::Configuration<IKKickConfig>& config) = 0;
		};

		class KickBalancer : public SixDOFFootController{
		private:
			//Config
			float stand_height = 0.18;
			float foot_separation = 0.074;
			float forward_lean = 0.01;
			float adjustment = 0.011;
			
		public:
			virtual void configure(const messages::support::Configuration<IKKickConfig>& config);
			virtual void computeStartMotion(const messages::input::Sensors& sensors);
			virtual void computeStopMotion(const messages::input::Sensors& sensors);

		};

		class Kicker : public SixDOFFootController{
		private:
			SixDOFFrame lift_foot;
			SixDOFFrame kick;
			SixDOFFrame place_foot;

			float follow_through;
			float wind_up;
			float foot_separation_margin;
		public:
			virtual void configure(const messages::support::Configuration<IKKickConfig>& config);
			virtual void computeStartMotion(const messages::input::Sensors& sensors);
			virtual void computeStopMotion(const messages::input::Sensors& sensors);
		};

	}
}

#endif