#pragma once
//#ifndef KASKADE_OPT_H
//#define KASKADE_OPT_H
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include "System.h"
#include "LocalMapping.h"

// #include "Map.h"
// #include "Frame.h"
// #include "KeyFrame.h"
// #include "Optimizer.h"
// #include "MapPoint.h"
// #include "LoopClosing.h"
// #include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"


#include "Trajectory.h"
#include <mutex>
#include <atomic>

#include <algorithm>
#include <vector>

namespace ORB_SLAM2
{
    class Map;
    class Frame;
    class KeyFrameDatabase;
    // class LocalMapping;
    class LoopClosing;
    class Optimizer;

    class System;

    class LocalMappingNonThread : public LocalMapping{

    public:
        using LocalMapping::LocalMapping;
        void NewKeyFrameInsertion(KeyFrame* pKF);
        KeyFrame* GetCurrentKeyFrame();
    };


    class KaskadeOptimizer 
    {
        public:
            KaskadeOptimizer(ORB_SLAM2::ORBVocabulary *voc, const float depthThr, const bool fixedScale);
            ~KaskadeOptimizer();

            void AddFrame(Frame pFr);//, std::vector<cv::Mat> poses);
            // void AddEnhancedFrame(Frame frEnh);
            // void AddKeyFrame(KeyFrame* pKF);

            void Optimize();
            void Reset();

            std::vector<KeyFrame*> GetTrajectoryKeyFrames();
            bool fixScale;

            int GetNumKeyFrames();
            int GetNumMapPoints();

            KeyFrame* CreateNewKeyFrame(Frame frame);//, std::vector<cv::Mat> poses);

            std::atomic_bool b_canAddNewFrames;

        private:
            ros::NodeHandle node_handle_;
            // cv::Mat getQuaternion(cv::Mat R);

            //TEST: Does this solve the issues with segmentation faults 
            //Local Mapper without a thread. It manages the local map and performs local bundle adjustment.
            //The Insertion And Optimization Processes are initiated manually instead of checking continually
            LocalMappingNonThread* mpKaskadeMapper;
            // std::thread*  mptKaskadeMapping;

            LoopClosing* mpLoopCloser_obs;
            std::thread* mptLoopClosing_obs;

            bool abortBA;

        public:
            Map *referenceMap;
            KeyFrameDatabase *keyFrameDatabase;
            Trajectory* trajectory;
            float mThDepth;
            mutex mMutexMapAcess;

            ros::Publisher pub_path;
    };
}
//#endif // KASKADE_OPT_H