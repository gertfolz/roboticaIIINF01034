#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include "Utils.h"

#define OCC_OCCUPIED 100
#define OCC_NEAROBSTACLE 90
#define OCC_FREE 50
#define OCC_FRONTIER 30
#define OCC_UNEXPLORED -1

#define PLAN_GOALS 0
#define PLAN_MARKEDGOALS 100
#define PLAN_REGULAR 40
#define PLAN_PATH 10
#define PLAN_INVALID -1

class Perception
{
public:
    Perception(ros::NodeHandle &n);

    bool hasValidDirection();
    double getDirectionOfNavigation();

private:
    void receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value);

    void updateCellsClassification();
    void computeHeuristic(int goalIndex);
    int computeShortestPathToFrontier(int robotIndex);

    void updateGridKnownLimits();
    int clusterFrontiersAndReturnIndexOfClosestOne(int robotIndex);
    void markPathCells(int goal);
    double computeDirectionOfNavigation(int robotIndex, int goalIndex);

    Pose2D getCurrentRobotPose();
    int getNearestFreeCell(Pose2D robotPose);

    bool started_;
    bool validDirection_;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;
    ros::Subscriber sub_gridmap_;
    ros::Publisher pub_mapOccType_, pub_mapPlanType_, pub_directionOfNavigation_;

    int numCellsX_;
    int numCellsY_;
    float mapWidth_;
    float mapHeight_;
    float scale_;

    int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;

    std::vector<int8_t> occupancyTypeGrid_;
    std::vector<int8_t> planningTypeGrid_;
    std::vector<double> fValueGrid_;
    std::vector<double> gValueGrid_;
    std::vector<double> hValueGrid_;
    std::vector<int> parentGrid_;

    std::vector<int> frontierCentersIndices;

    double directionOfNavigation_;

    nav_msgs::OccupancyGrid msg_occTypes_;
    nav_msgs::OccupancyGrid msg_planTypes_;
    geometry_msgs::PoseStamped msg_directionOfNavigation_;
};

#endif // PERCEPTION_H
