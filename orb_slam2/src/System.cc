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
#include "Converter.h"
#include <thread>
#include <iomanip>

#include <msgpack.hpp>
#include <string>
#include <iostream>
#include <sstream>

namespace ORB_SLAM2
{

System::System(const string strVocFile, const eSensor sensor, ORBParameters& parameters,
               const std::string & map_file, bool load_map): // map serialization addition
               mSensor(sensor), mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
               map_file(map_file), load_map(load_map)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "OpenCV version : " << CV_VERSION << endl;
    cout << "Major version : " << CV_MAJOR_VERSION << endl;
    cout << "Minor version : " << CV_MINOR_VERSION << endl;
    cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==DEEP_MONOCULAR)
        cout << "Monocular with aperiodic Depth information" << endl;

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary." << endl;

    mpVocabulary = new ORBVocabulary();

    //try to load from the binary file
    bool bVocLoad = mpVocabulary->loadFromBinFile(strVocFile+".bin");

    if(!bVocLoad)
    {
        cerr << "Cannot find binary file for vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile+".bin" << endl;
        cerr << "Trying to open the text file. This could take a while..." << endl;
        bool bVocLoad2 = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad2)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Failed to open at: " << strVocFile << endl;
            exit(-1);
        }
        cerr << "Saving the vocabulary to binary for the next time to " << strVocFile+".bin" << endl;
        mpVocabulary->saveToBinFile(strVocFile+".bin");
    }

    cout << "Vocabulary loaded!" << endl << endl;

    // begin map serialization addition
    // load serialized map
    if (load_map && LoadMap(map_file)) {
        std::cout << "Using loaded map with " << mpMap->MapPointsInMap() << " points\n" << std::endl;
    }
    else {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
        //Create the Map
        mpMap = new Map();
    }
    // end map serialization addition

    //For the purpose of MonoDepth KeyFrame Enhancement
    nKeyFrameEnhance = parameters.nKFEnhance;

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer,
                             mpMap, mpKeyFrameDatabase, mSensor, parameters);

    //Initialize the Local Mapping thread and launch
    bool bMONOLIKE = mSensor==MONOCULAR || mSensor==DEEP_MONOCULAR;
    mpLocalMapper = new LocalMapping(mpMap, bMONOLIKE);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    bool bFixScale = mSensor!=MONOCULAR && mSensor!=DEEP_MONOCULAR;
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, bFixScale);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the KaskadeOptimizer (thread and launch: TBD)
    auto pVoc = mpVocabulary;
    mpKaskadeOptimizer = new KaskadeOptimizer(pVoc, parameters.thDepth, bFixScale);
    
    // mptKaskadeOptimizer = new thread(&KaskadeOptimizer::Run, mpKaskadeOptimizer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    currently_localizing_only_ = false;
}

void System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
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
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
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

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
}

void System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
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
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
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

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
}

void System::TrackMonocular(const cv::Mat &im, const double &timestamp)
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
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
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

    current_position_ = Tcw;
}

bool System::timestamps_match (double timestamp1, double timestamp2){
    double diff = timestamp1 - timestamp2;
    if (diff < 0)
        diff = -diff;
    return diff < precision_enhanced_timestamp;
}

//This is the standard function to call when using an external depth model (i.e. pydnet)
void System::TrackDeepMono(const cv::Mat &im, const double &timestamp, const bool isEnhanced)
{
    eSensor mSensor_tmp = mSensor;
    mSensor = MONOCULAR;
    TrackMonocular(im, timestamp);
    mSensor = mSensor_tmp;

    // Frame curFrame = mpTracker -> mCurrentFrame;
    Frame curFrame = mpTracker -> mLastFrame;
    bool isPoseEmpty = curFrame.mTcw.empty();
    Frame frameToBeAdded;
    if(!isPoseEmpty){
        frameToBeAdded = Frame(curFrame);
        //std::cout << "Global Map MapPoints: " << mpMap->MapPointsInMap() << std::endl;
        //std::cout << "Frame MapPoints: " << frameToBeAdded.mvpMapPoints.size() << std::endl;

        // bool canAddNewFrame = mpKaskadeOptimizer -> b_canAddNewFrames;
        // std::cout << "Can add new frame: " << canAddNewFrame << std::endl;
        // if(canAddNewFrame)
        mpKaskadeOptimizer -> AddFrame(frameToBeAdded);//, refPoses);
    }

    //Check if is enhanced and update the current image accordingly
    if(isEnhanced && !isPoseEmpty){
        const std::lock_guard<std::mutex> lock(mMutexLastEnhancedFrame);
        last_enhanced_timestamp_ = curFrame.mTimeStamp;
        last_enhanced_frame_ = new Frame(frameToBeAdded);
        //mpKaskadeOptimizer -> b_canAddNewFrames = true;
    }else if (isEnhanced && isPoseEmpty){
        std::cout << "Pose is empty, cannot add new frame" << std::endl;
        last_enhanced_frame_ = nullptr;
        last_enhanced_timestamp_ = -1;
    }
    
    // else{
    //     last_enhanced_timestamp_ = -1;
    //     last_enhanced_frame_ = new Frame();
    // }

}

void System::TrackDeepDepth(const cv::Mat &im, const cv::Mat &depth, const double &timestamp){
/*
    std::cout << "TrackDeepDepth: called" << std::endl;
    bool matching_timestamps = true; timestamps_match(timestamp, last_enhanced_timestamp_);
    //std::cout << "Matching timestamps: " << matching_timestamps << "    Diff:" << timestamp -  last_enhanced_timestamp_ << std::endl;
    if(matching_timestamps){
        //Create a new frame with the image and the depth information


        mpKaskadeOptimizer -> b_canAddNewFrames = false;

        const std::lock_guard<std::mutex> lock(mMutexLastEnhancedFrame);
        bool isFrameNull = last_enhanced_frame_ == nullptr;
        
        if(isFrameNull){
            std::cout << "Frame is null" << std::endl;
            return;
        }
        else{
            
            Frame frameToBeAdded = Frame(*last_enhanced_frame_);       
            //Display the number of map points in frameToBeAdded
            frameToBeAdded.ComputeStereoFromRGBD(depth);
            //Destroy mutex lock_guard
            lock.~lock_guard();

            //Insert Enhanced Frame into the Kaskade Optimizer
            //mpKaskadeOptimizer -> AddEnhancedFrame(frameToBeAdded);
            mpKaskadeOptimizer -> AddFrame(frameToBeAdded);

            // std::cout << "Debug Checkpoint 2" << std::endl;

            std::cout << "Number of KeyFrames: " << mpKaskadeOptimizer -> GetNumKeyFrames() << std::endl;
            std::cout << "Number of MapPoints: " << mpKaskadeOptimizer -> GetNumMapPoints() << std::endl;
            
            // std::cout << "Debug Checkpoint 3" << std::endl;

            
            mpKaskadeOptimizer -> Optimize();
            std::cout << "Optimized" << std::endl;

            // std::cout << "Debug Checkpoint 4" << std::endl;

            mpKaskadeOptimizer -> Reset();
            std::cout << "Kaskade Optimizer reset." << std::endl;
        }

    }
*/
}



//This is the method used for the stereo case for the RAAD22 Paper
//A second image is also available
void System::TrackMonocularDepth_Stereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{   
    if(mSensor!=DEEP_MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocularDepth_Stereo but input sensor was not set to MONOCULAR_DEPTH." << endl;
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
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
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

    cv::Mat Tcw = mpTracker->GrabImageMonocularDepth_Stereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    current_position_ = Tcw;
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

bool System::isRunningGBA()
{
    return  mpLoopCloser->isRunningGBA();
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    ROS_INFO("Shutdown Initialized.");
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    ROS_INFO("Shutdown Request-Loop. Reached.");
    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    ROS_INFO("Shutdown Request-Loop. Finished.");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SetMinimumKeyFrames (int min_num_kf) {
  mpTracker->SetMinimumKeyFrames(min_num_kf);
}

cv::Mat System::GetCurrentPosition () {
  return current_position_;
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

cv::Mat System::DrawCurrentFrame () {
  return mpFrameDrawer->DrawFrame();
}

std::vector<MapPoint*> System::GetAllMapPoints() {
  return mpMap->GetAllMapPoints();
}


bool System::SetCallStackSize (const rlim_t kNewStackSize) {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return false;
    }

    if (kNewStackSize > rlimit.rlim_max) {
        std::cerr << "Requested call stack size too large" << std::endl;
        return false;
    }

    if (rlimit.rlim_cur <= kNewStackSize) {
        rlimit.rlim_cur = kNewStackSize;
        operation_result = setrlimit(RLIMIT_STACK, &rlimit);
        if (operation_result != 0) {
            std::cerr << "Setrlimit returned result: " << operation_result << std::endl;
            return false;
        }
        return true;
    }
    return false;
}


rlim_t System::GetCurrentCallStackSize () {
    struct rlimit rlimit;
    int operation_result;

    operation_result = getrlimit(RLIMIT_STACK, &rlimit);
    if (operation_result != 0) {
        std::cerr << "Error getting the call stack struct" << std::endl;
        return 16 * 1024L * 1024L; //default
    }

    return rlimit.rlim_cur;
}


void System::ActivateLocalizationMode()
{
    currently_localizing_only_ = true;
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    currently_localizing_only_ = false;
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::EnableLocalizationOnly (bool localize_only) {
  if (localize_only != currently_localizing_only_) {
    currently_localizing_only_ = localize_only;
    if (localize_only) {
      ActivateLocalizationMode();
    } else {
      DeactivateLocalizationMode();
    }
  }

  std::cout << "Enable localization only: " << (localize_only?"true":"false") << std::endl;
}


// map serialization addition
bool System::SaveMap(const string &filename) {
    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ofstream out(filename, std::ios_base::binary);
    if (!out) {
        std::cerr << "cannot write to map file: " << map_file << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    try {
        std::cout << "saving map file: " << map_file << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << mpMap;
        oa << mpKeyFrameDatabase;
        std::cout << " ... done" << std::endl;
        out.close();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    } catch (...) {
        std::cerr << "Unknows exeption" << std::endl;
        SetCallStackSize(kDefaultCallStackSize);
        return false;
    }

    SetCallStackSize(kDefaultCallStackSize);
    return true;
}

bool System::LoadMap(const string &filename) {

    unique_lock<mutex>MapPointGlobal(MapPoint::mGlobalMutex);
    std::ifstream in(filename, std::ios_base::binary);
    if (!in) {
        cerr << "Cannot open map file: " << map_file << " , you need create it first!" << std::endl;
        return false;
    }

    const rlim_t kNewStackSize = 64L * 1024L * 1024L;   // min stack size = 64 Mb
    const rlim_t kDefaultCallStackSize = GetCurrentCallStackSize();
    if (!SetCallStackSize(kNewStackSize)) {
        std::cerr << "Error changing the call stack size; Aborting" << std::endl;
        return false;
    }

    std::cout << "Loading map file: " << map_file << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    std::cout << " ... done" << std::endl;

    std::cout << "Map reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {

        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();

        if (it->mnFrameId > mnFrameId) {
            mnFrameId = it->mnFrameId;
        }
    }

    Frame::nNextId = mnFrameId;

    std::cout << " ... done" << std::endl;
    in.close();

    SetCallStackSize(kDefaultCallStackSize);

    return true;
}

} //namespace ORB_SLAM