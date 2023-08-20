#include "Perception.h"

#include <algorithm>
#include <chrono>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Perception::Perception(ros::NodeHandle& n):
    nh_(n)
{
    receivedMap_=false;
    startedMCL_=false;

    numParticles_=10000;
    maxRange_ = 10.0;

    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator_ = new std::default_random_engine(seed);

    // Initialize transform listener
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    // Initialize subscribers
    sub_laser_ = nh_.subscribe("/rosaria_phi/laser_laserscan", 100, &Perception::receiveLaser, this);
    sub_map_   = nh_.subscribe("/map", 100, &Perception::receiveGridmap, this);

    // Initialize publishers
    pub_particleFilter_ = nh_.advertise<geometry_msgs::PoseArray>("/particles", 1);

}

void Perception::MCL_sampling(const Action &u)
{
    /// TODO: propagar todas as particulas de acordo com o modelo de movimento baseado em odometria

    /// Odometria definida pela estrutura Action, composta por 3 variaveis double:
    /// rot1, trans e rot2
    std::cout << "rot1 " << RAD2DEG(u.rot1) << " trans " << u.trans << " rot2 " << RAD2DEG(u.rot2) << std::endl;

    /// Seguindo o modelo de Thrun, devemos gerar 3 distribuicoes normais, uma para cada componente da odometria

    /// Para definir uma distribuição normal X de media M e variancia V, pode-se usar:
    // std::normal_distribution<double> samplerX(M,V);
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerX(*generator_)
    /// onde *generator é um gerador de numeros aleatorios (definido no construtor da classe)

    /// Para acessar a i-ésima particula, usar:
    // particles_[i].p.x
    // particles_[i].p.y
    // particles_[i].p.theta










}

void Perception::MCL_weighting(const std::vector<float> &z)
{
   /// TODO: faça a pesagem de todas as particulas

    /// 1) elimine particulas fora do espaco livre, dando peso 0
    //       para achar a celula correspondente no grid, compute o indice dela
    //          unsigned int ix = particles_[i].p.x*scale_;
    //          unsigned int iy = particles_[i].p.y*scale_;
    //          unsigned int indice = ix + iy*numCellsX_;
    //       entao teste gridMap_.data[indice] <-- espaco livre tem valor 0

    /// 2) compare as observacoes da particula com as observacoes z do robo e pese-as
    //       Use a funcao computeExpectedMeasurement(k, particles[i].p)
    //       para achar a k-th observacao esperada da particula i
    ///    ao fim, normalize os pesos

 







}

void Perception::MCL_resampling()
{
    // gere uma nova geração de particulas com o mesmo tamanho do conjunto atual
    std::vector<Particle> nextGeneration;

    /// TODO: Implemente o Low Variance Resampling

    /// Para gerar amostras de uma distribuição uniforme entre valores MIN e MAX, pode-se usar:
    // std::uniform_real_distribution<double> samplerU(MIN,MAX));
    /// Para gerar amostras segundo a distribuicao acima, usa-se:
    // double amostra = samplerU(*generator_)
    /// onde *generator_ é um gerador de numeros aleatorios (definido no construtor da classe)







}

void Perception::MCL_initialize()
{

    unsigned int minKnownX_, minKnownY_, maxKnownX_, maxKnownY_;
    minKnownX_ = numCellsX_-1;
    minKnownY_ = numCellsY_-1;
    maxKnownX_ = maxKnownY_ = 0;

    // Update known limits
    for(unsigned int x=0; x<numCellsX_; x++){
        for(unsigned int y=0; y<numCellsY_; y++){
            unsigned int i = x + y*numCellsX_;
            if(gridMap_.data[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;

                if(gridMap_.data[i]>-1 && gridMap_.data[i]<90)
                    gridMap_.data[i]=0;
                else
                    gridMap_.data[i]=100;
            }
        }
    }

    particles_.resize(numParticles_);

    std::uniform_real_distribution<double> randomX(minKnownX_/scale_,maxKnownX_/scale_);
    std::uniform_real_distribution<double> randomY(minKnownY_/scale_,maxKnownY_/scale_);
    std::uniform_real_distribution<double> randomTh(-M_PI,M_PI);

    // generate initial particles set
    for(int i=0; i<numParticles_; i++){

        bool valid = false;
        do{
            // sample particle pose
            particles_[i].p.x = randomX(*generator_);
            particles_[i].p.y = randomY(*generator_);
            particles_[i].p.theta = randomTh(*generator_);

            // check if particle is valid (known and not obstacle)
            unsigned int ix = particles_[i].p.x*scale_;
            unsigned int iy = particles_[i].p.y*scale_;
            if(gridMap_.data[ix + iy*numCellsX_] == 0)
                valid = true;

        }while(!valid);

        std::cout << "Particle (" << i << "): "
                  << particles_[i].p.x << ' '
                  << particles_[i].p.y << ' '
                  << RAD2DEG(particles_[i].p.theta) << std::endl;
    }


    startedMCL_=true;
}

float Perception::computeExpectedMeasurement(int rangeIndex, Pose2D &pose)
{
    double angle = pose.theta + double(90-rangeIndex)*M_PI/180.0;

    // Ray-casting using DDA
    double dist;
    double difX=cos(angle);
    double difY=sin(angle);
    double deltaX, deltaY;

    if(tan(angle)==1 || tan(angle)==-1){
        deltaX=deltaY=1.0;
        dist = difX*maxRange_;
    }else if(difX*difX > difY*difY){
        deltaX=1.0;
        deltaY=difY/difX;
        dist = difX*maxRange_;
    }else{
        deltaX=difX/difY;
        deltaY=1.0;
        dist = difY*maxRange_;
    }

    if(deltaX*difX < 0.0)
        deltaX = -deltaX;
    if(deltaY*difY < 0.0)
        deltaY = -deltaY;
    if(dist < 0.0)
        dist = -dist;

    dist *= scale_;

    double i=pose.x*scale_;
    double j=pose.y*scale_;
    for(int k=0;k<(int)(dist);k++){


        unsigned int ix = i;
        unsigned int iy = j;
        if(gridMap_.data[(int)i + (int)j*numCellsX_] == 100){
            // the real obstacle is one step ahead due to wall thickening
            return sqrt(pow(pose.x*scale_-(i+deltaX),2)+pow(pose.y*scale_-(j+deltaY),2))/scale_;
        }

        i+=deltaX;
        j+=deltaY;
    }

    return maxRange_;
}

unsigned int msg_count=0;
void Perception::MCL_publishParticles()
{
    geometry_msgs::PoseArray msg_particles;

    msg_particles.header.frame_id="map";
    msg_particles.header.seq = msg_count++;
    msg_particles.header.stamp = ros::Time::now();

    msg_particles.poses.resize(numParticles_);
    for(unsigned int i=0; i<numParticles_; i++){
        msg_particles.poses[i].position.x = particles_[i].p.x + mapOrigin_.x;
        msg_particles.poses[i].position.y = particles_[i].p.y + mapOrigin_.y;

        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, particles_[i].p.theta );
        msg_particles.poses[i].orientation = tf2::toMsg(quat_tf);
    }

    pub_particleFilter_.publish(msg_particles);
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

const std::vector<float>& Perception::getLaserReadings()
{
    return lasersROS_;
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

    if(lasersROS_.size()<numLasers)
        lasersROS_.resize(numLasers);

    for(int i=0; i<numLasers; i++){
        lasersROS_[i] = value->ranges[numLasers-i-1];
        if(lasersROS_[i]<0 || lasersROS_[i]>maxRange_)
            lasersROS_[i] = maxRange_;
    }
}

void Perception::receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value)
{
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

    if(receivedMap_==false){
        gridMap_.header = value->header;
        gridMap_.info = value->info;
        gridMap_.data = value->data;

        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;
        scale_ = 1.0/cellSize;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;

        mapOrigin_.x = value->info.origin.position.x;
        mapOrigin_.y = value->info.origin.position.y;

        receivedMap_=true;
    }

}

bool Perception::hasReceivedMap()
{
    return receivedMap_;
}

bool Perception::hasStartedMCL()
{
    return startedMCL_;
}