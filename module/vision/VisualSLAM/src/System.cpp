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
#include "message/input/Image.h"
//#include "Converter.h"
#include <iomanip>


namespace module {
namespace vision {

	using message::input::Image;
	using message::input::CameraParameters;

	System::System(const std::string &strVocFile, const eSensor sensor)
				  :mSensor(sensor), mbReset(false),mbActivateLocalizationMode(false),
				  mbDeactivateLocalizationMode(false)
	{
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
	    //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

	    //Create the Map
	    //mpMap = new Map();

	    //Initialize Tracking
	    //mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
	    //                         mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

	    //Initialize Local Mapping
	    //mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);

	    //Initialize Loop Closing
	    //mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
	}


}  // namespace vision
}  // namespace module	