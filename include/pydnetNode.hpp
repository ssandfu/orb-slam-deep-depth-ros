/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

THIS IS NOT FUNCTIONAL CODE

PLEASE USE THE PYTHON VERSION IN THE FOLDER "nodes"

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

#ifndef PYDNET_ROS_H_
#define PYDNET_ROS_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "cppflow/ops.h"
#include "cppflow/model.h"

class PyDNetNode
{
  public:
    PyDNetNode (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~PyDNetNode ();
    void Init ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msg);
  protected:
    ros::Time current_frame_time_;
    std::string camera_info_topic_;
    std::string name_of_node_;
    //ros::NodeHandle node_handle_;
    //image_transport::ImageTransport image_transport_;
  private:
    image_transport::Subscriber image_subscriber;
    ros::Publisher depthimage_pub;

    //cppflow::model depth_pred_model;
};

#endif //PYDNET_ROS_H_