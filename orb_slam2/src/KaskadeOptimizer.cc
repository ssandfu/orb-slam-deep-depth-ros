#include "KaskadeOptimizer.hpp"

namespace ORB_SLAM2
{

    void LocalMappingNonThread::NewKeyFrameInsertion(KeyFrame* pKF){
        InsertKeyFrame(pKF);
        if(CheckNewKeyFrames()){
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Triangulate new MapPoints
            CreateNewMapPoints();

            // if(!CheckNewKeyFrames())
            // {
            //     // Find more matches in neighbor keyframes and fuse point duplications
            //     SearchInNeighbors();
            // }

            // Check redundant local Keyframes
            // KeyFrameCulling();

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
        }else{
            std::cout << "Something weird happend during the insertion of a new KeyFrame into an object of the type LocalMappingNonThread." << std::endl;
        }
    }

    KeyFrame* LocalMappingNonThread::GetCurrentKeyFrame(){
        return mpCurrentKeyFrame;
    }

    KaskadeOptimizer::KaskadeOptimizer(ORB_SLAM2::ORBVocabulary *voc, const float depthThr, const bool fixedScale):
        mThDepth(depthThr), fixScale(fixedScale), b_canAddNewFrames(false), abortBA(false)
    {
        referenceMap = new Map();
        keyFrameDatabase = new KeyFrameDatabase(*voc);
        trajectory = new Trajectory();
        node_handle_ = ros::NodeHandle("~");
        pub_path = node_handle_.advertise<nav_msgs::Path>("/kaskade_optimizer/path", 1);

        //Initialize the Local Mapping thread and launch
        bool bMONOLIKE = true; //mSensor==MONOCULAR || mSensor==DEEP_MONOCULAR;
        mpKaskadeMapper = new LocalMappingNonThread(referenceMap, bMONOLIKE);
        // mptKaskadeMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpKaskadeMapper);

        //Initialize the Loop Closing thread and launch
        bool bFixScale = false;
        mpLoopCloser_obs = new LoopClosing(referenceMap, keyFrameDatabase, voc, bFixScale);
        mptLoopClosing_obs = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser_obs);

        mpKaskadeMapper->SetLoopCloser(mpLoopCloser_obs);

        b_canAddNewFrames = true;
    }

    KaskadeOptimizer::~KaskadeOptimizer()
    {
        delete referenceMap;
        delete keyFrameDatabase;
        delete trajectory;
    }

    // void KaskadeOptimizer::AddEnhancedFrame(Frame frEnh){
    //     const std::lock_guard<std::mutex> lock(mMutexMapAcess);
    //     //b Find the matching frame for frEnh.mTimeStamp
        // std::vector<KeyFrame*> kfs = keyFrameDatabase->GetAllKeyFrames();
        // KeyFrame* kf = NULL;
        // for(int i=0; i<kfs.size(); i++){
        //     if(kfs[i]->mTimeStamp == frEnh.mTimeStamp){
        //         kf = kfs[i];
        //         break;
        //     }
        // }
        //If no matching frame is found, return nothing and warn the user.
        // if(kf == NULL){
        //     std::cout << "KaskadeOptimizer::AddEnhancedFrame: No matching keyframe found for frame " << frEnh.mTimeStamp << std::endl;
        //     return;
        // }
        //If a matching frame is found, replace the existing frame with frEnh
        
        //Delete all references to the old frame

        //Add the new enhanced frame to the map
        
    // }

    void KaskadeOptimizer::AddFrame(Frame pFr)//, std::vector<cv::Mat> poses)
    {
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        // Create new keyframe
        KeyFrame* pKF = CreateNewKeyFrame(pFr);//, poses);
        if(b_canAddNewFrames == false)
            std::cout << "Trying to access Frames during Optimization." << std::endl;
        //mpKaskadeMapper->InsertKeyFrame(pKF);
        mpKaskadeMapper->NewKeyFrameInsertion(pKF);
        std::cout << "KeyFrame inserted and processed in KaskadeMapper" << std::endl;
        
        /*
        // Compute Bags of Words structures
        pKF->ComputeBoW();
        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<MapPoint*> vpMapPointMatches = pKF->GetMapPointMatches();
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(pKF))
                    {
                        pMP->AddObservation(pKF, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                        referenceMap->AddMapPoint(pMP);
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        //mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }
        // Update links in the Covisibility Graph
        pKF->UpdateConnections();

        std::cout << "Added KeyFrame " << pKF->mnId << " with " << (pKF->GetMapPointMatches()).size() << " MapPointMatches "<< "and " << (pKF->GetConnectedKeyFrames()).size() << " KeyFrame Connections" << std::endl;
        */
        /*
        //Add KeyFrame to Map, Database and trajectory
        referenceMap->AddKeyFrame(pKF);
        keyFrameDatabase->add(pKF);
        trajectory->AddKeyFrame(pKF);

        const long nMPs_map = (referenceMap->GetAllMapPoints()).size();
        const long nKFs_map = (referenceMap->GetAllKeyFrames()).size();

        std::cout << "MapPoints in Reference Map: " << nMPs_map << std::endl;
        std::cout << "Keyframes in Reference Map: " << nKFs_map << std::endl;
        */
    }

    void KaskadeOptimizer::Optimize()
    {
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        b_canAddNewFrames = false;
        bool* abortBA = new bool(false);
        KeyFrame* curKF = mpKaskadeMapper->GetCurrentKeyFrame();
        std::cout << "Starting Local Mapping Optimizer" <<std::endl;
        Optimizer::LocalBundleAdjustment(curKF, abortBA, referenceMap);
        std::cout << "Kaskade Local Mapping Optimizer done" << std::endl;
        b_canAddNewFrames = true;


        /*
        b_canAddNewFrames = false;
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);

        //Get all relevant keyframes and the assoziated MapPoints from the database
        std::vector<KeyFrame*> relevantKeyFrames = referenceMap -> GetAllKeyFrames();
        std::vector<MapPoint*> relevantMapPoints = referenceMap -> GetAllMapPoints();

        //Get KeyFrame IDs in Referenze Map
        std::cout << "Keyframes in Reference Map: ";
        std::vector<int> kfIds;
        for(int i=0; i<relevantKeyFrames.size(); i++){
            kfIds.push_back(relevantKeyFrames[i]->mnId);
            std::cout << " " << relevantKeyFrames[i]->mnId;
        }
        std::cout << std::endl;

        //Clean up the Mappoints so that only keyframes in this (very) local map are used 
        //and no errors are thrown by the optimization
        for(auto it = relevantMapPoints.begin(); it != relevantMapPoints.end(); it++){
            
            auto observations = (*it)->GetObservations();
            for(auto obs = observations.begin(); obs != observations.end(); obs++){
                int id = obs->first->mnId;
                if(std::find(kfIds.begin(), kfIds.end(), id) != kfIds.end()) {
                    //kfIds contains id
                    //Pass
                } else {
                    //kfIds does not contain id
                    observations.erase(obs);
                }

            }
            if( observations.size() == 0){
                std::cout << "MapPoint " << (*it)->mnId << " has no observations. Deleting..." <<std::endl;
            }else{
                //Get Observant KeyFrame IDs
                std::cout << "MapPoint " << (*it)->mnId << " has " << observations.size() << " observations";
                std::cout << "(";
                std::vector<int> obsKfIds;
                // map<KeyFrame*,size_t> observations = (*it)->GetObservations();
                //observations = (*it)->GetObservations();
                for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                {
                    int id = mit->first->mnId;
                    obsKfIds.push_back(id);
                    std::cout << " " << id;
                }
                std::cout << ")" << std::endl;
            }
        }


        //Optimize the pose of each keyframe
        std::vector<KeyFrame*> kaskadedKeyFrames;
        for(int i=0; i<relevantKeyFrames.size(); i++){
            std::cout << "Adress relKF[" << i << "]: " << &relevantKeyFrames[i] << std::endl;
            if(std::find(kfIds.begin(), kfIds.end(), relevantKeyFrames[i]->mnId) != kfIds.end()){
                kaskadedKeyFrames.push_back(relevantKeyFrames[i]);
                std::cout << "Adress kaskKF[" << i << "]: " << &kaskadedKeyFrames[i] << std::endl;
                std::cout << "Pose before optimization: " << kaskadedKeyFrames[i]->GetPose() << std::endl;
                std::cout << "Sample MapPoint: " << relevantMapPoints[i] -> GetWorldPos() << std::endl;

                try{
                    const std::vector<KeyFrame*> vKF = kaskadedKeyFrames;
                    const vector<MapPoint *> vMP = relevantMapPoints;
                    Optimizer::BundleAdjustment(vKF, vMP);
                }catch(const std::exception &exc)
                {
                    // catch anything thrown within try block that derives from std::exception
                    std::cout << exc.what();
                }
                std::cout << "Pose after  optimization: " << kaskadedKeyFrames[i]->GetPose() << std::endl;
            }
        }
        */

        //Execute a global bundle adjustment
        // Optimizer::GlobalBundleAdjustemnt(referenceMap, fixScale);
        // kaskadedKeyFrames = referenceMap -> GetAllKeyFrames();
        // std::cout << "Optimized" << std::endl;

        //Create a path message from all keyframes in the map
        nav_msgs::Path path = nav_msgs::Path();
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();
        
        std::vector<KeyFrame*> kaskadedKeyFrames = referenceMap -> GetAllKeyFrames();
        for(int i=0; i < kaskadedKeyFrames.size(); i++){
            //Get original cv matrix format pose from keyframe
            cv::Mat translationKF = kaskadedKeyFrames[i]->GetTranslation();
            cv::Mat rot = kaskadedKeyFrames[i]->GetRotation().clone();

            // Convert into tf datatypes
            tf2::Vector3 transTF(translationKF.at<float>(0,0), translationKF.at<float>(1,0), translationKF.at<float>(2,0));
            tf2::Matrix3x3 tf2_rot(rot.at<float>(0, 0), rot.at<float>(0, 1), rot.at<float>(0, 2),
                                   rot.at<float>(1, 0), rot.at<float>(1, 1), rot.at<float>(1, 2),
                                   rot.at<float>(2, 0), rot.at<float>(2, 1), rot.at<float>(2, 2));
            // Create a tf2 transform and convert to ROS pose msg
            geometry_msgs::Pose poseTF; 
            tf2::Transform tf2_transform(tf2_rot, transTF);
            tf2::toMsg(tf2_transform, poseTF);            

            //Change the Pose msg into a PoseStamped msg by adding a header 
            geometry_msgs::PoseStamped poseROS = geometry_msgs::PoseStamped();
            poseROS.header.stamp = ros::Time(kaskadedKeyFrames[i]->mTimeStamp);
            poseROS.pose = poseTF;

            //Add the pose to the path
            path.poses.push_back(poseROS);
        }

        //Publish the path
        pub_path.publish(path);
        std::cout << "Path is published" << std::endl;
        b_canAddNewFrames = true;
    }

    void KaskadeOptimizer::Reset()
    {
        b_canAddNewFrames = false;
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        // mpKaskadeMapper->RequestReset();
/*
        //Reset the Database and trajectory to an empty state for the next optimization
        keyFrameDatabase -> clear();
        trajectory -> Clear();
        referenceMap -> clear();
        std::cout << "Kaskade Optimizer Reset." << std::endl;
*/ 
        b_canAddNewFrames = true;
    }

    std::vector<KeyFrame*> KaskadeOptimizer::GetTrajectoryKeyFrames()
    {
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        return trajectory->GetAllKeyFrames();
    }

    int KaskadeOptimizer::GetNumKeyFrames(){
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        return (referenceMap -> GetAllKeyFrames()).size();
    }
    int KaskadeOptimizer::GetNumMapPoints(){
        const std::lock_guard<std::mutex> lock(mMutexMapAcess);
        return (referenceMap -> GetAllMapPoints()).size();
    }

    KeyFrame* KaskadeOptimizer::CreateNewKeyFrame(Frame frame)//, std::vector<cv::Mat> poses)
    {
        std::cout << "KaskadeOptimizer::CreateNewKeyFrame called" << std::endl;
        // if(!mpLocalMapper->SetNotStop(true))
        //     return;
        KeyFrame* pKF = new KeyFrame(frame, referenceMap, keyFrameDatabase);
        // std::cout << "New KeyFrame created" << std::endl;

        // const int n_poses = mpTracker->mCurrentFrame.mvpMapPoints.size();
        // std::vector<cv::Mat> refPoses;
        // for(int i = 0; i < n_poses; i++){
        //     if(mpTracker->mCurrentFrame.mvpMapPoints[i] != nullptr){
        //         cv::Mat pose = mpTracker->mCurrentFrame.mvpMapPoints[i]->GetWorldPos();
        //         refPoses.push_back(pose);
        //         mpKaskadeOptimizer-> referenceMap -> AddMapPoint(mpTracker->mCurrentFrame.mvpMapPoints[i]);
        //     }else
        //         refPoses.push_back(cv::Mat());
        // }

/*
        const int nPoses = frame.mvpMapPoints.size();
        std::cout << "Number of poses: " << nPoses << std::endl;
        for(int i = 0; i < nPoses; i++){
            if(frame.mvpMapPoints[i] != nullptr){
                MapPoint* pMP = new MapPoint(frame.mvpMapPoints[i] -> GetWorldPos(), pKF, referenceMap);
                referenceMap -> AddMapPoint(pMP);
                pKF -> AddMapPoint(pMP, i);
            }
        }
        std::cout << "MapPoints added to KeyFrame" << std::endl;
*/

        frame.mpReferenceKF = pKF;
        // auto mSensor = System::DEEP_MONOCULAR;
        // bool isNotMono = mSensor!=System::MONOCULAR;
        // bool isNotMonoDepth = (mSensor != System::DEEP_MONOCULAR);
        // bool isRealMonoDepth = ((mSensor == System::DEEP_MONOCULAR) && (frame.mpORBextractorRight != static_cast<ORBextractor*>(NULL)));
        // if(isNotMono && (isNotMonoDepth || isRealMonoDepth) )
        // {
            frame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float,int> > vDepthIdx;
            vDepthIdx.reserve(frame.N);
            for(int i=0; i<frame.N; i++)
            {
                float z = frame.mvDepth[i];
                if(z>0)
                {
                    vDepthIdx.push_back(make_pair(z,i));
                }
            }

            if(!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(),vDepthIdx.end());

                int nPoints = 0;
                for(size_t j=0; j<vDepthIdx.size();j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint* pMP = frame.mvpMapPoints[i];
                    if(!pMP)
                        bCreateNew = true;
                    else if(pMP->Observations()<1)
                    {
                        bCreateNew = true;
                        frame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                    }

                    if(bCreateNew)
                    {
                        cv::Mat x3D = frame.UnprojectStereo(i);
                        MapPoint* pNewMP = new MapPoint(x3D, pKF, referenceMap);
                        pNewMP->AddObservation(pKF,i);
                        pKF->AddMapPoint(pNewMP,i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        referenceMap->AddMapPoint(pNewMP);

                        frame.mvpMapPoints[i]=pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if(vDepthIdx[j].first>mThDepth && nPoints>100)
                        break;
                }
            }
        // }
        pKF->UpdateConnections();

        // mpLocalMapper->InsertKeyFrame(pKF);
        // mpLocalMapper->SetNotStop(false);
        // mnLastKeyFrameId = mCurrentFrame.mnId;
        // mpLastKeyFrame = pKF;
        return pKF;
    }


} // namespace ORB_SLAM2