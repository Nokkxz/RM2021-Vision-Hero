#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <thread>
#include <sstream>
#include "csignal"

class Watchdog
{
public:
  Watchdog()
  {
    timer_camera_ = nh_.createTimer(ros::Duration(2), &Watchdog::timerCameraCallback, this);
    sub_camera_ = nh_.subscribe("watchdog_camera", 10, &Watchdog::subCameraCallback, this);
  }

  void timerCameraCallback(const ros::TimerEvent&)
  {
    if(cameraFlag_ == 0)
    {
      ROS_INFO("Watchdog: Camera Failed");
      system("gnome-terminal -x rosrun roborts_camera roborts_camera_node");
      ros::Duration(4).sleep();
      // system("gnome-terminal -x rosrun roborts_detection armor_detection_node");
      // system("gnome-terminal -x rosrun roborts_detection armor_detection_client");
    }
    else
    {
      ROS_INFO("Watchdog: Camera OK");
    }
    cameraFlag_ = 0;
  }

  void subCameraCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    cameraFlag_ = msg->data;
    // ROS_INFO("cameraCallback receive: [%d]", cameraFlag_);
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_camera_;
  ros::Subscriber sub_camera_;

  int8_t cameraFlag_;

};

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "roborts_watchdog_node");
  Watchdog watchdog_test;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}]
