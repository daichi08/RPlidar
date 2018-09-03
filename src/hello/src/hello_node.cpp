#include "ros/ros.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "hello_node");
  ros::NodeHandle n;
  
  ros::Time begin = ros::Time::now();
  ros::Rate loop_rate(10);
  
  float time = 0;
  
  while(ros::ok()){    
    ros::Duration duration = ros::Time::now()-begin;
    
    double secs = duration.toSec();
    ROS_INFO("ROS: %u.%u", duration.sec, duration.nsec);
    ROS_INFO("%lf", secs);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
