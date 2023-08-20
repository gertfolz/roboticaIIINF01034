#include "Perception.h"

Perception::Perception()
{

}

void Perception::receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value)
{
//  STRUCTURE OF sensor_msgs::LaserScan

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis
    laserROS.header = value->header;

    //float32 angle_min        # start angle of the scan [rad]
    //float32 angle_max        # end angle of the scan [rad]
    //float32 angle_increment  # angular distance between measurements [rad]
    laserROS.angle_min = value->angle_min;
    laserROS.angle_max = value->angle_max;
    laserROS.angle_increment = value->angle_increment;

    //float32 time_increment   # time between measurements [seconds] - if your scanner
    //                         # is moving, this will be used in interpolating position
    //                         # of 3d points
    //float32 scan_time        # time between scans [seconds]
    laserROS.time_increment = value->time_increment;
    laserROS.scan_time = value->scan_time;

    //float32 range_min        # minimum range value [m]
    //float32 range_max        # maximum range value [m]
    laserROS.range_min = value->range_min;
    laserROS.range_max = value->range_max;

    //float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    //float32[] intensities    # intensity data [device-specific units].  If your
    //                         # device does not provide intensities, please leave
    //                         # the array empty.
    laserROS.ranges = value->ranges;
    laserROS.intensities = value->intensities;
}

void Perception::receiveSonar(const sensor_msgs::PointCloud::ConstPtr &value)
{
//  STRUCTURE OF sensor_msgs::PointCloud

    //# This message holds a collection of 3d points, plus optional additional
    //# information about each point.

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    sonarROS.header = value->header;

    //  # Array of 3d points. Each Point32 should be interpreted as a 3d point
    //  # in the frame given in the header.
    //geometry_msgs/Point32[] points
    sonarROS.points = value->points;

    //  # Each channel should have the same number of elements as points array,
    //  # and the data in each channel should correspond 1:1 with each point.
    //  # Channel names in common practice are listed in ChannelFloat32.msg.
    //ChannelFloat32[] channels
    sonarROS.channels = value->channels;
}

std::vector<float> Perception::getLatestLaserRanges(){
    int numLasers = laserROS.ranges.size();

    std::vector<float> lasers(numLasers);

//    std::cout << "LASER: " << numLasers << std::endl;
    for(int i=0; i<numLasers; i++){
        lasers[i] = laserROS.ranges[numLasers-i-1];
        if(lasers[i]<0)
            lasers[i] = 32.0; //max range from rosaria
    }
    
    return lasers;
}

std::vector<float> Perception::getLatestSonarRanges(){
    int numSonars = sonarROS.points.size();

    std::vector<float> sonars(numSonars);

//    std::cout << "SONAR: " << numSonars << std::endl;
    for(int i=0; i<numSonars; i++){
//        std::cout << RAD2DEG(atan2(sonarROS.points[i].y,sonarROS.points[i].x)) << ' ';
        sonars[i] = sqrt(pow(sonarROS.points[i].x,2.0)+pow(sonarROS.points[i].y,2.0));
    }
//    std::cout << std::endl;
    
    return sonars;
}


