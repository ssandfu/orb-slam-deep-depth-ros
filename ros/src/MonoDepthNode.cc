#include "MonoDepthNode.h"

//Only on the n-th keyframe the stereo image will be used. In tracking no stereo information will be used.
//The use stereo information in tracking set this value to 0. This leads to every keyframe getting stereo information
//Other cases are not intended at the moment
const int nStereoEnhanced_glob = 0;
const int nKFEnhanced_glob = 4;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MonoDepth");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoDepthNode node (ORB_SLAM2::System::MONOCULAR_DEPTH, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoDepthNode::MonoDepthNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : 
                              Node (sensor, node_handle, image_transport) {
  depthimage_pub = node_handle.advertise<sensor_msgs::Image>("/delayed_depth_info", 1);

  image_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  right_sub_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/image_right/image_color_rect", 1);  
  
  node_handle_.getParam(name_of_node_ + "/nKFEnhance", nKFEnhanced);
  std::cout << "KF Enhanced set to: " << nKFEnhanced << std::endl;
  node_handle_.getParam(name_of_node_ + "/nStereoDelay", nStereoEnhanced);
  std::cout << "nStereoEnhanced set to: " << nStereoEnhanced << std::endl;
  //ROS_WARN("CAUTION: the parameter nStereoDelay has no effect with the current implementation.");


  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *image_sub_, *right_sub_);

  if(nKFEnhanced > 0 && nStereoEnhanced == 0){
    sync_->registerCallback(boost::bind(&MonoDepthNode::ImageKeyFrameDelayCallback, this, _1, _2));
    std::cout << "Callback registered to ImageKeyFrameDelayCallback" << std::endl;
  }
  else if(nKFEnhanced == 0 && nStereoEnhanced > 0){
    sync_->registerCallback(boost::bind(&MonoDepthNode::ImageDelayCallback, this, _1, _2));
        std::cout << "Callback registered to ImageDelayCallback" << std::endl;
  }
  else
    std::cerr << "Please set either nKFEnhanced or nStereoEnhanced to the desired value and the other one to 0." << std::endl;
  
  //image_subscriber = image_transport.subscribe ("/camera/rgb/image_raw", 1, &MonoDepthNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";

}


MonoDepthNode::~MonoDepthNode () {
}


void MonoDepthNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocularDepth(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update();
}

void MonoDepthNode::ImageKeyFrameDelayCallback (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
  //static int callback_cntr = 0;
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgLeft->header.stamp;
  //depthimage_pub.publish(msgRight);
  orb_slam_->TrackMonocularDepth_Stereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
  //ROS_INFO("TrackStereo");
  Update ();
  //callback_cntr++;
}

void MonoDepthNode::ImageDelayCallback (const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
  static int callback_cntr = 0;
  cv_bridge::CvImageConstPtr cv_ptrLeft;
  try {
      cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrRight;
  try {
      cv_ptrRight = cv_bridge::toCvShare(msgRight);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msgLeft->header.stamp;

  //nStereoEnhanced_glob is (uncleanly) declared and defined as a constant global variable at the top of the file. 
  if(callback_cntr % nStereoEnhanced != 0){
    //ROS_INFO("TrackMono");
    orb_slam_->TrackMonocularDepth(cv_ptrLeft->image,cv_ptrLeft->header.stamp.toSec());
  }
  else{
    //ROS_INFO("TrackStereo");
    depthimage_pub.publish(msgRight);
    orb_slam_->TrackMonocularDepth_Stereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
  }
  Update ();
  callback_cntr++;
}
