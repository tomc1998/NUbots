/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "System.h"
#include "stdio.h"
#include "utility/math/vision.h"
#include "Converter.h"
#include <iomanip>


namespace module {
namespace vision {

	using message::input::Image;
	using message::input::CameraParameters;

	void System::Initialize(const std::string &strVocFile,const std::string &strSettingsFile, const eSensor sensor)
	{
		mSensor = sensor;
		mbReset = false;
		mbActivateLocalizationMode = false;
		mbDeactivateLocalizationMode = false;
	    // Output welcome message
	    std::cout << std::endl <<
	    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << std::endl <<
	    "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
	    "This is free software, and you are welcome to redistribute it" << std::endl <<
	    "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

	    std::cout << "Input sensor was set to: ";

	    if(mSensor==MONOCULAR)
	        std::cout << "Monocular" << std::endl;
	    else if(mSensor==STEREO)
	        std::cout << "Stereo" << std::endl;

	    //Load ORB Vocabulary
	    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

	    mpVocabulary = new ORBVocabulary();
	    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	    if(!bVocLoad)
	    {
	        cerr << "Wrong path to vocabulary. " << std::endl;
	        cerr << "Failed to open at: " << strVocFile << std::endl;
	        exit(-1);
	    }
	    std::cout << "Vocabulary loaded!" << std::endl << std::endl;
		
	    //Create KeyFrame Database
	    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

	    //Create the Map
	    mpMap = new Map();

	    //Initialize Tracking
	    mpTracker = new Tracking(this, mpVocabulary, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

	    //Initialize Local Mapping
	    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);

	    //Initialize Loop Closing
	    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);

	    //Set pointers between threads
	    mpTracker->SetLocalMapper(mpLocalMapper);
	    mpTracker->SetLoopClosing(mpLoopCloser);

	    mpLocalMapper->SetTracker(mpTracker);
	    mpLocalMapper->SetLoopCloser(mpLoopCloser);

	    mpLoopCloser->SetTracker(mpTracker);
	    mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	// Launch LocalMapping
	void System::launchLocalMapping()
	{
		std::cout << "Launched Local Mapping" << std::endl;
		mpLocalMapper->Run();
	}

    // Launch LoopClosing
    void System::launchLoopClosing()
    {
    	std::cout << "Launched Loop Closing" << std::endl;
    	mpLoopCloser->Run();
    }

	

	cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
	{
	    if(mSensor!=MONOCULAR)
	    {
	        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
	        exit(-1);
	    }

	    // Check mode change
	    {
	        unique_lock<mutex> lock(mMutexMode);
	        if(mbActivateLocalizationMode)
	        {
	            mpLocalMapper->RequestStop();

	            // Wait until Local Mapping has effectively stopped
	            while(!mpLocalMapper->isStopped())
	            {
	                usleep(1000);
	            }

	            mpTracker->InformOnlyTracking(true);
	            mbActivateLocalizationMode = false;
	        }
	        if(mbDeactivateLocalizationMode)
	        {
	            mpTracker->InformOnlyTracking(false);
	            mpLocalMapper->Release();
	            mbDeactivateLocalizationMode = false;
	        }
	    }

	    // Check reset
	    {
	    unique_lock<mutex> lock(mMutexReset);
	    if(mbReset)
	    {
	        mpTracker->Reset();
	        mbReset = false;
	    }
	    }

	    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

	    unique_lock<mutex> lock2(mMutexState);
	    mTrackingState = mpTracker->mState;
	    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
	    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

	    return Tcw;
	}

	void System::ActivateLocalizationMode()
	{
	    unique_lock<mutex> lock(mMutexMode);
	    mbActivateLocalizationMode = true;
	}

	void System::DeactivateLocalizationMode()
	{
	    unique_lock<mutex> lock(mMutexMode);
	    mbDeactivateLocalizationMode = true;
	}

	bool System::MapChanged()
	{
	    static int n=0;
	    int curn = mpMap->GetLastBigChangeIdx();
	    if(n<curn)
	    {
	        n=curn;
	        return true;
	    }
	    else
	        return false;
	}

	void System::Reset()
	{
	    unique_lock<mutex> lock(mMutexReset);
	    mbReset = true;
	}

	
	void System::Shutdown()
	{
	    mpLocalMapper->RequestFinish();
	    mpLoopCloser->RequestFinish();

	    // Wait until all thread have effectively stopped
	    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
	    {
	        usleep(5000);
	    }
	}
	

	int System::GetTrackingState()
	{
	    unique_lock<mutex> lock(mMutexState);
	    return mTrackingState;
	}

	vector<MapPoint*> System::GetTrackedMapPoints()
	{
	    unique_lock<mutex> lock(mMutexState);
	    return mTrackedMapPoints;
	}

	vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
	{
	    unique_lock<mutex> lock(mMutexState);
	    return mTrackedKeyPointsUn;
	}



}  // namespace vision
}  // namespace module	