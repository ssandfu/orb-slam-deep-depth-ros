/*
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

THIS IS NOT FUNCTIONAL CODE

PLEASE USE THE PYTHON VERSION IN THE FOLDER "nodes"

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

#include "pydnetNode.hpp"

void PyDNetNode::Init () {
  //depth_pred_model = cppflow::model("checkpoint/IROS18/", cppflow::model::TYPE::FROZEN_GRAPH);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pydNet");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    PyDNetNode node (node_handle, image_transport);
    node.Init();

    ros::spin();
    ros::shutdown();

    return 0;
}


PyDNetNode::PyDNetNode (ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport){
  image_subscriber = image_transport.subscribe ("/kitti/camera_color_left/image_rect", 1, &PyDNetNode::ImageCallback, this);
  camera_info_topic_ = "/kitti/camera_color_left/camera_info";
  std::cout << "Node initialized." << std::endl;
}


PyDNetNode::~PyDNetNode () {
}


void PyDNetNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  current_frame_time_ = cv_in_ptr->header.stamp;
  std::cout << "Image received. Timestamp:  " << current_frame_time_ << std::endl;
  //cv_in_ptr->image,cv_in_ptr->header.stamp.toSec()
  //auto output = depth_pred_model(cv_in_ptr->image);
}
