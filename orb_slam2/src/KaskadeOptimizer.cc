#include "KaskadeOptimizer.hpp"

namespace ORB_SLAM2
{
    KaskadeOptimizer::KaskadeOptimizer(const ORBVocabulary *voc, const float depthThr, const bool fixedScale):
        mThDepth(depthThr), fixScale(fixedScale)
    {
        referenceMap = new Map();
        keyFrameDatabase = new KeyFrameDatabase(*voc);
        trajectory = new Trajectory();
    }

    KaskadeOptimizer::~KaskadeOptimizer()
    {
        delete referenceMap;
        delete keyFrameDatabase;
        delete trajectory;
    }

    void KaskadeOptimizer::AddFrame(Frame pFr)//, std::vector<cv::Mat> poses)
    {
        // Create new keyframe
        KeyFrame* pKF = CreateNewKeyFrame(pFr);//, poses);
        referenceMap->AddKeyFrame(pKF);
        keyFrameDatabase->add(pKF);
        trajectory->AddKeyFrame(pKF);
        
        const long nMPs_map = (referenceMap->GetAllMapPoints()).size();
        const long nKFs_map = (referenceMap->GetAllKeyFrames()).size();

        std::cout << "MapPoints in Reference Map: " << nMPs_map << std::endl;
        std::cout << "Keyframes in Reference Map: " << nKFs_map << std::endl;

    }

    void KaskadeOptimizer::Optimize()
    {
        //Get all relevant keyframes and the assoziated MapPoints from the database
        std::vector<KeyFrame*> relevantKeyFrames = referenceMap -> GetAllKeyFrames();
        std::vector<MapPoint*> relevantMapPoints = referenceMap -> GetAllMapPoints();

        std::cout << "Optimized" << std::endl;

    }

    void KaskadeOptimizer::Reset()
    {
        //Reset the Database and trajectory to an empty state for the next optimization
        keyFrameDatabase -> clear();
        trajectory -> Clear();
        referenceMap -> clear();
        std::cout << "Kaskade Optimizer Reset." << std::endl;
    }

    std::vector<KeyFrame*> KaskadeOptimizer::GetTrajectoryKeyFrames()
    {
        return trajectory->GetAllKeyFrames();
    }

    int KaskadeOptimizer::GetNumKeyFrames(){
        return (referenceMap -> GetAllKeyFrames()).size();
    }
    int KaskadeOptimizer::GetNumMapPoints(){
        return (referenceMap -> GetAllMapPoints()).size();
    }

    KeyFrame* KaskadeOptimizer::CreateNewKeyFrame(Frame frame)//, std::vector<cv::Mat> poses)
    {
        std::cout << "KaskadeOptimizer::CreateNewKeyFrame called" << std::endl;
        auto mSensor = System::DEEP_MONOCULAR;
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

        frame.mpReferenceKF = pKF;
        bool isNotMono = mSensor!=System::MONOCULAR;
        bool isNotMonoDepth = (mSensor != System::DEEP_MONOCULAR);
        bool isRealMonoDepth = ((mSensor == System::DEEP_MONOCULAR) && (frame.mpORBextractorRight != static_cast<ORBextractor*>(NULL)));
        if(isNotMono && (isNotMonoDepth || isRealMonoDepth) )
        {
            //std::cout << "Entered IF" << std::endl;
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
        }
        pKF->UpdateConnections();

        // mpLocalMapper->InsertKeyFrame(pKF);
        // mpLocalMapper->SetNotStop(false);
        // mnLastKeyFrameId = mCurrentFrame.mnId;
        // mpLastKeyFrame = pKF;
        return pKF;
    }


} // namespace ORB_SLAM2