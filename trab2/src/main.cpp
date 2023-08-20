#include <ros/ros.h>

#include "Perception.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mapping");
    ROS_INFO("mapping");

    ros::NodeHandle n("~");
    ros::Rate rate(5); // run 5 times per second

    Perception perception(n, 100, 100, 0.1);

    ros::spin();
   
    return 0;
}
