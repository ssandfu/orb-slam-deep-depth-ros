#include "DeepMonoNode.h"

//Only on the n-th keyframe the stereo image will be used. In tracking no stereo information will be used.
//The use stereo information in tracking set this value to 0. This leads to every keyframe getting stereo information
//Other cases are not intended at the moment
const int nStereoEnhanced_glob = 0;
const int nKFEnhanced_glob = 4;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DeepMonoNode");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    //DeepMonoNode node (ORB_SLAM2::System::RGBD , node_handle, image_transport);
    DeepMonoNode node (ORB_SLAM2::System::DEEP_MONOCULAR , node_handle, image_transport);
    //DeepMonoNode node (ORB_SLAM2::System::MONOCULAR , node_handle, image_transport);

    node.Init();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    // ros::spin();

    ros::shutdown();

    return 0;
}


DeepMonoNode::DeepMonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : 
                              Node (sensor, node_handle, image_transport) {
  left_image_subscriber = image_transport::Subscriber(image_transport.subscribe("/camera/rgb/image_raw", 1, &DeepMonoNode::ImageCallback_Mono, this));
  deep_depth_image_subscriber = image_transport::Subscriber(image_transport.subscribe("/depth_network_image", 1, &DeepMonoNode::DeepDepthCallback, this));
  
  network_task_publisher = image_transport::Publisher(image_transport.advertise("/depth_model_task_image", 1));  
  heartbeat_publisher_ = node_handle.advertise<orb_slam_deep_depth_ros::heartbeat>("/DeepMono/heartbeat", 1000);

  // publishHeartBeat();

  node_handle_.getParam(name_of_node_ + "/nKFEnhance", nKFEnhanced);
  std::cout << "KF Enhanced set to: " << nKFEnhanced << std::endl;
  node_handle_.getParam(name_of_node_ + "/nStereoDelay", nStereoEnhanced);
  std::cout << "nStereoEnhanced set to: " << nStereoEnhanced << std::endl;
  ROS_WARN("CAUTION: the parameter nKFEnhanced and nStereoEnhanced have no effect with the current implementation.");

  publish_heartbeat_depth = true;
  publish_heartbeat_mono = true;

}


DeepMonoNode::~DeepMonoNode () {
}

// void DeepMonoNode::publishHeartBeat () {
//   //publish the heartbeat

//   if (publish_heartbeat_mono & publish_heartbeat_depth){
//     orb_slam_deep_depth_ros::heartbeat s_msg;
//     s_msg.header.stamp = ros::Time::now();
//     s_msg.mono_beat = true;
//     s_msg.depth_beat = false;
//     heartbeat_publisher_.publish(s_msg);    
//   }else{
//     ROS_WARN("Heartbeat can not be published.");
//   }

// }

void DeepMonoNode::ImageCallback_Mono (const sensor_msgs::ImageConstPtr& msgRGB) {
  static int frameCounter = 0;
  
  publish_heartbeat_mono = false;

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  current_frame_time_ = msgRGB->header.stamp;

  bool isEnhanced = (frameCounter++ % nStereoEnhanced == 0);

  orb_slam_->TrackDeepMono(cv_ptrRGB->image, cv_ptrRGB->header.stamp.toSec(), isEnhanced);
  // std::cout << "Monocular Image was grabbed at: " << current_frame_time_.toSec() << std::endl;
  
  if (isEnhanced) {
    network_task_publisher.publish(msgRGB);
    std::cout << "Monocular Image was published at: " << current_frame_time_.toSec() << " for Depth estimation." << std::endl;
    cv_RBG_Matrix_enhancement = (cv_ptrRGB->image).clone();
  }
  Update();

  // publish_heartbeat_mono = true;
  // publishHeartBeat();
}

void DeepMonoNode::DeepDepthCallback (const sensor_msgs::ImageConstPtr& msgRGB) {

  publish_heartbeat_depth = false;

  cv_bridge::CvImageConstPtr cv_ptrDepth;
  try {
      cv_ptrDepth = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  ROS_INFO("Depth image was grabbed at: %f", cv_ptrDepth->header.stamp.toSec());
  orb_slam_->TrackDeepDepth(cv_RBG_Matrix_enhancement, cv_ptrDepth->image, cv_ptrDepth->header.stamp.toSec());
  
  // publish_heartbeat_depth = true;
  // publishHeartBeat();
}



void DeepMonoNode::ImageCallback_RGBD (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgRGB->header.stamp;
  ROS_INFO("Callback called.");
  orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
  ROS_INFO("RGBD Image was grabbed at: %f", current_frame_time_.toSec());
  Update ();
}