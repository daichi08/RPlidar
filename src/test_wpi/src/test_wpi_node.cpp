#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "wiringPi.h"

#define RAD2DEG(x) ((x)*180./M_PI)

float distance = 0;
float angle;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        if(90.0 <= degree || degree <= -90.0){
            //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
            if(distance == 0){
                distance = scan->ranges[i];
                angle =  degree;
            }else{
                if(distance > scan->ranges[i]){
                    distance = scan->ranges[i];
                    angle =  degree;
                }
            }
        }
    }
    ROS_INFO(": [%f, %f]", distance, angle);
    distance = 0;
    angle = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    wiringPiSetup();
    pinMode(1, OUTPUT);

    ros::spin();

    return 0;
}
