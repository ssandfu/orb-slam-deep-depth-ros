#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "KeyFrame.h"
#include <vector>


namespace ORB_SLAM2
{
    struct TrajectoryPoint {
        cv::Mat Pose;
        double TimeStamp;
    };

    class Trajectory
    {
        public:
            Trajectory();
            ~ Trajectory();
            void AddKeyFrame(KeyFrame* pKF);
            void Clear();

            std::vector<KeyFrame*> GetAllKeyFrames();
            std::vector<TrajectoryPoint> GetTrajectoryPoints();

            std::vector<TrajectoryPoint> GetTrajectoryPoints_Sorted();

        private:
            std::vector<KeyFrame*> frames;
            std::vector<TrajectoryPoint> trajectory;

            static bool CompareTimeStamps(TrajectoryPoint p1, TrajectoryPoint p2){
                return p1.TimeStamp < p2.TimeStamp;
            }

    };
}

#endif // TRAJECTORY_H