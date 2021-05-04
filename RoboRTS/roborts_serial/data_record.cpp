#include <ros/ros.h>
#include "roborts_msgs/RelativeAngle.h"
#include "roborts_msgs/DataRecord.h"
#include <thread>
#include <sstream>
#include "csignal"

class DataRecord
{
public:
  DataRecord()
  {
    relative_angle_ = 0.0;

    timer_ = nh_.createTimer(ros::Duration(1), &DataRecord::timerCallback, this);
    sub_ = nh_.subscribe("roborts_relative_angle", 1000, &DataRecord::subCallback, this);
    pub_ = nh_.advertise<roborts_msgs::DataRecord>("roborts_data_record",1000);
    // pub_ = nh_.advertise<roborts_msgs::RelativeAngle>("roborts_relative_angle",1000);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    relative_angle_++;
    if(relative_angle_ > 100) relative_angle_ = 0.0;
    roborts_msgs::DataRecord msg;
    msg.relative_angle = relative_angle_;
    pub_.publish(msg);
    ROS_INFO("RelativeAngle send: [%f]", relative_angle_);
  }

  void subCallback(const roborts_msgs::RelativeAngle& msg)
  {
    relative_angle_ = msg.relative_angle;
    ROS_INFO("RelativeAngle receive: [%f]", relative_angle_);
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  float relative_angle_;

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
  ros::init(argc, argv, "roborts_data_record_node");
  DataRecord dataRecord;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
