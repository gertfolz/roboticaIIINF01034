#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <thread>     
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "Utils.h"

class Perception
{
public:
    Perception(ros::NodeHandle& n, float mapWidth, float mapHeight, float cellSize);

private:

    void updateMapLaserWithHIMM(const std::vector<float>& z);
    void updateMapLaserWithLogOdds(const std::vector<float>& z);
    void updateMapSonarWithHIMM(const std::vector<float>& z);

    void receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);
    void receiveSonar(const sensor_msgs::PointCloud::ConstPtr &value);

    Pose2D getCurrentRobotPose();
    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);

    int getCellIndexFromXY(int x, int y);

    ros::NodeHandle nh_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    ros::Subscriber sub_laser_, sub_sonar_;
    ros::Publisher pub_mapLaserLogOdds_;
    ros::Publisher pub_mapLaserHIMM_; 
    ros::Publisher pub_mapSonarHIMM_;

    int numCellsX_;
    int numCellsY_;

    float mapWidth_;
    float mapHeight_;
    float scale_;

    std::vector<float> gridLaserLogOdds_;
    std::vector<int> gridLaserHIMM_;
    std::vector<int> gridSonarHIMM_;

    nav_msgs::OccupancyGrid msg_mapLaserLogOdds_;
    nav_msgs::OccupancyGrid msg_mapLaserHIMM_;
    nav_msgs::OccupancyGrid msg_mapSonarHIMM_;
};

#endif //PERCEPTION_H
