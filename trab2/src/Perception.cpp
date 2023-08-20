#include "Perception.h"
#include <chrono>
#include <algorithm>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Perception::Perception(ros::NodeHandle& n, float mapWidth, float mapHeight, float cellSize):
    nh_(n), mapWidth_(mapWidth), mapHeight_(mapHeight), scale_(1.0/cellSize)
{
    // Initialize transform listener
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    // Initialize publishers
    pub_mapLaserLogOdds_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_laser_log_odds", 1);
    pub_mapLaserHIMM_    = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_laser_HIMM", 1);
    pub_mapSonarHIMM_   = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_sonar_HIMM", 1);
 
    // Initialize subscribers
    sub_laser_ = nh_.subscribe("/rosaria_phi/laser_laserscan", 100, &Perception::receiveLaser, this);
    sub_sonar_ = nh_.subscribe("/rosaria_phi/sonar", 100, &Perception::receiveSonar, this);

    // Initialize grids
    numCellsX_ = mapWidth_*scale_;
    numCellsY_ = mapHeight_*scale_;

    gridLaserLogOdds_.resize(numCellsX_*numCellsY_, 0); // default value = log  0.5/(1-0.5) = 0
    gridLaserHIMM_.resize(numCellsX_*numCellsY_, 7);    // default value = 7 (half between 0 and 15)
    gridSonarHIMM_.resize(numCellsX_*numCellsY_, 7);    // default value = 7 (half between 0 a 15)

    // Initialize OccupancyGrid messages

    // STRUCTURE OF nav_msgs::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    msg_mapLaserHIMM_.header.frame_id="odom";
    msg_mapLaserHIMM_.info.height = numCellsY_;
    msg_mapLaserHIMM_.info.width = numCellsX_;
    msg_mapLaserHIMM_.info.resolution = cellSize;
    msg_mapLaserHIMM_.info.origin.position.x = -mapWidth/2.0;
    msg_mapLaserHIMM_.info.origin.position.y = -mapHeight/2.0;
    msg_mapLaserHIMM_.data.resize(numCellsX_*numCellsY_, -1);


    msg_mapLaserLogOdds_.header.frame_id="odom";
    msg_mapLaserLogOdds_.info = msg_mapLaserHIMM_.info;
    msg_mapLaserLogOdds_.data.resize(numCellsX_*numCellsY_, -1);

    msg_mapSonarHIMM_.header.frame_id="odom";
    msg_mapSonarHIMM_.info = msg_mapLaserHIMM_.info;
    msg_mapSonarHIMM_.data.resize(numCellsX_*numCellsY_, -1);

}

/////////////////////////////////////////////////
/// Callbacks dos topicos do LASER e do SONAR ///
/////////////////////////////////////////////////

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
    //    string frame_id;
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis

    //float32 angle_min        # start angle of the scan [rad]
    //float32 angle_max        # end angle of the scan [rad]
    //float32 angle_increment  # angular distance between measurements [rad]

    //float32 time_increment   # time between measurements [seconds] - if your scanner
    //                         # is moving, this will be used in interpolating position
    //                         # of 3d points
    //float32 scan_time        # time between scans [seconds]

    //float32 range_min        # minimum range value [m]
    //float32 range_max        # maximum range value [m]

    //float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    //float32[] intensities    # intensity data [device-specific units].  If your
    //                         # device does not provide intensities, please leave
    //                         # the array empty.

    int numLasers = value->ranges.size();

    std::vector<float> lasers(numLasers);

//    std::cout << "LASER: " << numLasers << std::endl;
    for(int i=0; i<numLasers; i++){
        lasers[i] = value->ranges[numLasers-i-1];
        if(lasers[i]<0)
            lasers[i] = 32.0; //max range from rosaria
    }

    msg_mapLaserHIMM_.header.stamp = value->header.stamp;
    updateMapLaserWithHIMM(lasers);
    msg_mapLaserLogOdds_.header.stamp = value->header.stamp;
    updateMapLaserWithLogOdds(lasers);

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

    //  # Array of 3d points. Each Point32 should be interpreted as a 3d point
    //  # in the frame given in the header.
    //geometry_msgs/Point32[] points

    //  # Each channel should have the same number of elements as points array,
    //  # and the data in each channel should correspond 1:1 with each point.
    //  # Channel names in common practice are listed in ChannelFloat32.msg.
    //ChannelFloat32[] channels

    int numSonars = value->points.size();

    std::vector<float> sonars(numSonars);

//    std::cout << "SONAR: " << numSonars << std::endl;
    for(int i=0; i<numSonars; i++){
//        std::cout << RAD2DEG(atan2(value->points[i].y,value->points[i].x)) << ' ';
        sonars[i] = sqrt(pow(value->points[i].x,2.0)+pow(value->points[i].y,2.0));
    }
//    std::cout << std::endl;

    msg_mapSonarHIMM_.header.stamp = value->header.stamp;
    updateMapSonarWithHIMM(sonars);
}

/////////////////////////////
/// Funcoes de mapeamento ///
/////////////////////////////

void Perception::updateMapLaserWithLogOdds(const std::vector<float>& z)
{
    Pose2D robot = getCurrentRobotPose(); // x,y in meters, theta in degrees
    std::cout << robot.x << ' ' << robot.y << ' ' << robot.theta << std::endl;

    int rx = robot.x*scale_;
    int ry = robot.y*scale_;

    float maxRange = 10.0; // 10 m
    float lambda_r = 0.2; //  20 cm
    float lambda_phi = 1.0;  // 1 degree
    int maxRangeInt = maxRange*scale_;

    float locc, lfree;
    
    // TODO:
    // varrer celulas ao redor de (rx,ry) em um range de -maxRangeInt ate +maxRangeInt nas duas direcoes
    // e entao atualizar celulas dentro do campo de visao dos sensores 
    
    // para acessar/atualizar ocupacao da i-esima celula do mapa usar:
    // gridLaserLogOdds_[i]
    // onde i eh o indice da celula u,v, que pode ser obtido com a funcao auxiliar i=getCellIndexFromXY(u,v);
    
    // para visualizar corretamente no rviz, sempre atualizar msg_mapLaserLogOdds_.data[i]
    // importante lembrar de converter o valor de log-odds para OccupancyGrid data (que vai de 0 a 100)
    // Dica: converter primeiro para probabilidades usando a funcao getLikelihoodFromLogOdds() e multiplicar por 100
    // ex: msg_mapLaserLogOdds_.data[i] = getLikelihoodFromLogOdds(gridLaserLogOdds_[i])*100;

    for (int u = rx - maxRangeInt; u <= rx + maxRangeInt; u++)
    {
        for (int v = ry - maxRangeInt; v <= ry + maxRangeInt; v++)
        {
            int i = getCellIndexFromXY(u, v);
        
            // Compute the distance r to the cell where the robot is
            float r = sqrt((u-rx)*(u-rx) + (v-ry)*(v-ry));

            // Compute the orientation phi of the cell in local coordinates
            float phi = atan2(v-ry, u-rx) * 180.0 / M_PI - robot.theta;
            
            // Normalize the angles between -180 and 180 degrees
            phi = normalizeAngleDEG(phi);
            // Find the nearest laser beam to the orientation phi
            int k = getNearestLaserBeam(phi);
            // std::cout << ' ' << x << ' ' << y << ' ' << r << ' ' << phi << ' ' << k << std::endl;
            r=r/scale_;
            // Check if the cell is within the field of view of the laser sensor
            if (abs(normalizeAngleDEG(phi - getAngleOfLaserBeam(k))) <= lambda_phi/2 && r <= std::min(maxRange, z[k] + lambda_r / 2))
            {
                // std::cout << "within field of view" << std::endl;
                // Check if the cell is close enough to the measurement
                // std::this_thread::sleep_for (std::chrono::milliseconds(10));
                // std::cout << r << ' ' << z[k] << std::endl; // converte o R
                if (std::abs(r - z[k]) < lambda_r/2)
                {
                    locc = getLogOddsFromLikelihood(0.8);
                    gridLaserLogOdds_[i] += locc;
                    double p = getLikelihoodFromLogOdds(gridLaserLogOdds_[i]) * 100;
                    msg_mapLaserLogOdds_.data[i] = p;
                }
                else if (r <= z[k])
                {
                    lfree = getLogOddsFromLikelihood(0.3);
                    gridLaserLogOdds_[i] += lfree;
                    double p = getLikelihoodFromLogOdds(gridLaserLogOdds_[i]) * 100;
                    msg_mapLaserLogOdds_.data[i] = p;
                }
            }
        }
    }
    pub_mapLaserLogOdds_.publish(msg_mapLaserLogOdds_);    
}

void Perception::updateMapLaserWithHIMM(const std::vector<float>& z)
{
    Pose2D robot = getCurrentRobotPose(); // x,y in meters, theta in degrees

    int rx = robot.x*scale_;
    int ry = robot.y*scale_;

    float maxRange = 10.0; // 10 m
    float lambda_r = 0.2; //  20 cm
    float lambda_phi = 1.0;  // 1 degree
    int maxRangeInt = maxRange*scale_;

    // TODO:
    // varrer celulas ao redor de (rx,ry) em um range de -maxRangeInt ate +maxRangeInt nas duas direcoes
    // e entao atualizar celulas dentro do campo de visao dos sensores 
    
    // para acessar/atualizar ocupacao da i-esima celula do mapa usar:
    // gridLaserHIMM_[i]
    // onde i eh o indice da celula u,v, que pode ser obtido com a funcao auxiliar i=getCellIndexFromXY(u,v);
    
    // para visualizar corretamente no rviz, sempre atualizar msg_mapLaserHIMM_.data[i]
    // importante lembrar de converter o valor, originalmente de 0 a 15, para OccupancyGrid data (que vai de 0 a 100)
        for (int u = rx - maxRangeInt; u <= rx + maxRangeInt; u++)
    {
        for (int v = ry - maxRangeInt; v <= ry + maxRangeInt; v++)
        {
            int i = getCellIndexFromXY(u, v);
            float r = sqrt((u-rx)*(u-rx) + (v-ry)*(v-ry));
            float phi = atan2(v-ry, u-rx) * 180.0 / M_PI - robot.theta;
            phi = normalizeAngleDEG(phi);
            int k = getNearestLaserBeam(phi);
            // std::cout << ' ' << x << ' ' << y << ' ' << r << ' ' << phi << ' ' << k << std::endl;
            r=r/scale_;
            // Check if the cell is within the field of view of the laser sensor
            if (abs(normalizeAngleDEG(phi - getAngleOfLaserBeam(k))) <= lambda_phi/2 && r <= std::min(maxRange, z[k] + lambda_r / 2))
            {
                if (std::abs(r - z[k]) < lambda_r/2)
                {
                    gridLaserHIMM_[i] += 3;
                    // Limitar o valor máximo de ocupação em 15
                    if (gridLaserHIMM_[i] > 15)
                        gridLaserHIMM_[i] = 15;
                    double p = gridLaserHIMM_[i] * 100 / 15.0; 
                    msg_mapLaserHIMM_.data[i] = p;
                }
                else if (r <= z[k])
                {
                    gridLaserHIMM_[i]--;
                    // Limitar o valor mínimo de ocupação em 0
                    if (gridLaserHIMM_[i] < 0)
                        gridLaserHIMM_[i] = 0;
                    double p = gridLaserHIMM_[i] * 100 / 15.0; 
                    msg_mapLaserHIMM_.data[i] = p;
                }
            }
        }
    }
    pub_mapLaserHIMM_.publish(msg_mapLaserHIMM_);
}

void Perception::updateMapSonarWithHIMM(const std::vector<float>& z)
{
    Pose2D robot = getCurrentRobotPose(); // x,y in meters, theta in degrees

    int rx = robot.x*scale_;
    int ry = robot.y*scale_;

    float maxRange = 5.0;
    int maxRangeInt = maxRange*scale_;

    float lambda_r = 0.3; 
    float lambda_phi = 2.0;


    // TODO:
    // varrer celulas ao redor de (rx,ry) em um range de -maxRangeInt ate +maxRangeInt nas duas direcoes
    // e entao atualizar celulas dentro do campo de visao dos sensores 
    // devido a menor quantidade de sensores no sonar, menos celulas serao atualizadas
    // Dica: usar um angulo maior para celulas proximas ao robo e um angulo menor nos outros casos 
    
    // para acessar/atualizar ocupacao da i-esima celula do mapa usar:
    // gridSonarHIMM_[i]
    // onde i eh o indice da celula u,v, que pode ser obtido com a funcao auxiliar i=getCellIndexFromXY(u,v);
    
    // para visualizar corretamente no rviz, sempre atualizar msg_mapSonarHIMM_.data[i]
    // importante lembrar de converter o valor, originalmente de 0 a 15, para OccupancyGrid data (que vai de 0 a 100)

    for (int u = rx - maxRangeInt; u <= rx + maxRangeInt; u++)
    {
        for (int v = ry - maxRangeInt; v <= ry + maxRangeInt; v++)
        {
            int i = getCellIndexFromXY(u, v);
            float r = sqrt((u-rx)*(u-rx) + (v-ry)*(v-ry));
            float phi = atan2(v-ry, u-rx) * 180.0 / M_PI - robot.theta;

            phi = normalizeAngleDEG(phi);

            int k = getNearestSonarBeam(phi);
            r=r/scale_;
            if (abs(normalizeAngleDEG(phi - getAngleOfSonarBeam(k))) <= lambda_phi/2 && r <= std::min(maxRange, z[k] + lambda_r / 2))
            {
                if (std::abs(r - z[k]) < lambda_r/2)
                {
                    gridSonarHIMM_[i] += 3;
                    if (gridSonarHIMM_[i] > 15)
                        gridSonarHIMM_[i] = 15;
                    double p = gridSonarHIMM_[i] * 100 / 15.0;
                    msg_mapSonarHIMM_.data[i] = p;

                    // Growth Rate Operator
                    for (int du = -1; du <= 1; du++)
                    {
                        for (int dv = -1; dv <= 1; dv++)
                        {
                            int neighborIndex = getCellIndexFromXY(u + du, v + dv);
                            if (neighborIndex != i)
                            {
                                gridSonarHIMM_[neighborIndex] += 1;
                                if (gridSonarHIMM_[neighborIndex] > 15)
                                    gridSonarHIMM_[neighborIndex] = 15;
                                double p = gridSonarHIMM_[neighborIndex] * 100 / 15.0;
                                msg_mapSonarHIMM_.data[neighborIndex] = p;
                            }
                        }
                    }
                }
                else if (r <= z[k])
                {
                    gridSonarHIMM_[i]--;
                    // Limitar o valor mínimo de ocupação em 0
                    if (gridSonarHIMM_[i] < 0)
                        gridSonarHIMM_[i] = 0;
                    double p = gridSonarHIMM_[i] * 100 / 15.0;
                    msg_mapSonarHIMM_.data[i] = p;
                }
            }
        }
    }
    pub_mapSonarHIMM_.publish(msg_mapSonarHIMM_);   
}

//////////////////////////
/// Funcoes Auxiliares ///
//////////////////////////

int Perception::getCellIndexFromXY(int x, int y)
{
    int cellX = numCellsX_/2 + x;
    int cellY = numCellsY_/2 + y;

    return cellX + numCellsX_*cellY;
}

Pose2D Perception::getCurrentRobotPose()
{
    Pose2D robotPose;

    // Get robot transformation given by ODOM
    geometry_msgs::TransformStamped transformStamped;
    try{ transformStamped = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s",ex.what()); }
    
    robotPose.x = transformStamped.transform.translation.x;
    robotPose.y = transformStamped.transform.translation.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(transformStamped.transform.rotation.x,
                       transformStamped.transform.rotation.y, 
                       transformStamped.transform.rotation.z, 
                       transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}

float sonarAngles_[8] = {90, 50, 30, 10, -10, -30, -50, -90};

float Perception::getAngleOfSonarBeam(int k)
{
    return sonarAngles_[k];
}

int Perception::getNearestSonarBeam(float angle)
{
    if(angle>70.0)
        return 0;
    else if(angle>40 && angle<=70)
        return 1;
    else if(angle>20 && angle<=40)
        return 2;
    else if(angle>0 && angle<=20)
        return 3;
    else if(angle>-20 && angle<=0)
        return 4;
    else if(angle>-40 && angle<=-20)
        return 5;
    else if(angle>-70 && angle<=-40)
        return 6;
    else //if(angle<=-70.0)
        return 7;

}

float Perception::getAngleOfLaserBeam(int k)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    return 90.0-(float)k;
}

int Perception::getNearestLaserBeam(float angle)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    if(angle>90.0)
        return 0;
    else if(angle<-90.0)
        return 180;
    else{
        return 90-(int)((angle > 0.0)?(angle + 0.5):(angle - 0.5));
    }
}
