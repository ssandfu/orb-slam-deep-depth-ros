#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::MultiThreadedSpinner spinner(4);

    spinner.spin();

    // while(ros::ok){
    //   node.checkIsLocalMapIdle();
    //   node.publishHeartBeat();
    //   spinner.spin();
    // }

    // ros::spin();

    ros::shutdown();

    return 0;
}

void MonoNode::checkIsLocalMapIdle(){
  // publish_heartbeat_map_idle = orb_slam_->mpTracker->canPublishHeartbeat();
  publish_heartbeat_map_idle = orb_slam_->mpLocalMapper->AcceptKeyFrames();
}

void MonoNode::publishHeartBeat () {
  //publish the heartbeat

  if (publish_heartbeat_map_idle){
    orb_slam_deep_depth_ros::heartbeat s_msg;
    s_msg.header.stamp = ros::Time::now();
    s_msg.mono_beat = true;
    s_msg.depth_beat = true;
    heartbeat_publisher_.publish(s_msg);    
  }
  // else{
  //   ROS_WARN("Heartbeat can not be published.");
  // }

}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 3, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";
  heartbeat_publisher_ = node_handle.advertise<orb_slam_deep_depth_ros::heartbeat>("/DeepMono/heartbeat", 1000);

  publish_heartbeat_map_idle = true;

}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;

  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();
}
