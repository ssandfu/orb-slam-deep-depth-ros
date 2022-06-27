#include "Trajectory.h"

namespace ORB_SLAM2
{
    Trajectory::Trajectory(){
        frames = std::vector<KeyFrame*>();
    }

    Trajectory::~Trajectory(){
        Clear();
    }

    void Trajectory::Clear(){
        frames.clear();
    }

    void Trajectory::AddKeyFrame(KeyFrame* pKF){
        frames.push_back(pKF);
    }

    std::vector<KeyFrame*> Trajectory::GetAllKeyFrames(){
        return frames;
    }

    std::vector<TrajectoryPoint> Trajectory::GetTrajectoryPoints(){
        std::vector<TrajectoryPoint> trajectory;
        for(int i = 0; i < frames.size(); i++){
            TrajectoryPoint point;
            point.Pose = frames[i]->GetPose();
            point.TimeStamp = frames[i]->mTimeStamp;
            trajectory.push_back(point);
        }
        return trajectory;
    }

    std::vector<TrajectoryPoint> Trajectory::GetTrajectoryPoints_Sorted(){
        std::vector<TrajectoryPoint> trajectory = Trajectory::GetTrajectoryPoints();
        std::sort(trajectory.begin(), trajectory.end(), Trajectory::CompareTimeStamps);
        return trajectory;
    }
}