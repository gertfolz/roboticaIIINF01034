#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

#include <random>

#include "Utils.h"

class Perception
{
public:
    Perception(ros::NodeHandle& n);

    void MCL_initialize();

    void MCL_sampling(const Action &u);
    void MCL_weighting(const std::vector<float> &z);
    void MCL_resampling();

    void MCL_publishParticles();

    bool hasReceivedMap();
    bool hasStartedMCL();

    Pose2D getCurrentRobotPose();
    const std::vector<float>& getLaserReadings();

private:

    void receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);
    void receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value);

    float computeExpectedMeasurement(int index, Pose2D &pose);

    ros::NodeHandle nh_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener* tfListener_;

    ros::Publisher pub_particleFilter_;

    ros::Subscriber sub_laser_, sub_map_;
    std::vector<float> lasersROS_;
    nav_msgs::OccupancyGrid gridMap_;

    bool receivedMap_;
    bool startedMCL_;

    int numCellsX_;
    int numCellsY_;
    float mapWidth_;
    float mapHeight_;
    float scale_;
    Pose2D mapOrigin_;
    float maxRange_;

    int numParticles_;
    std::vector<Particle> particles_;
    std::default_random_engine* generator_;
};

#endif //PERCEPTION_H
