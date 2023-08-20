#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

class Perception
{
public:
    Perception();
    
    std::vector<float> getLatestLaserRanges();
    std::vector<float> getLatestSonarRanges();
    
    void receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);
    void receiveSonar(const sensor_msgs::PointCloud::ConstPtr &value);

private:
    sensor_msgs::LaserScan laserROS;
    sensor_msgs::PointCloud sonarROS;

};

#endif // PERCEPTION_H
