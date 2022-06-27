#ifndef KASKADE_OPTIMIZER_H
#define KASKADE_OPTIMIZER_H

#include "Map.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
// #include "Optimizer.h"

#include "Trajectory.h"


namespace ORB_SLAM2
{
    class Map;
    class KeyFrameDatabase;
    class Optimizer;



    class KaskadeOptimizer
    {
        public:
            KaskadeOptimizer(const ORBVocabulary *voc, const float depthThr, const bool fixedScale);
            ~KaskadeOptimizer();

            void AddFrame(Frame pFr);//, std::vector<cv::Mat> poses);
            // void AddKeyFrame(KeyFrame* pKF);

            void Optimize();
            void Reset();

            std::vector<KeyFrame*> GetTrajectoryKeyFrames();
            bool fixScale;

            int GetNumKeyFrames();
            int GetNumMapPoints();

            KeyFrame* CreateNewKeyFrame(Frame frame);//, std::vector<cv::Mat> poses);


        public:
            Map *referenceMap;
            KeyFrameDatabase *keyFrameDatabase;
            Trajectory* trajectory;
            float mThDepth;
    };
}
#endif // KASKADE_OPTIMIZER_H
