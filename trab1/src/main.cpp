#include <pthread.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include "Action.h"
#include "Perception.h"
#include "Utils.h"

char pressedKey;

void* keyboardThreadFunction(void* arg)
{
    while(pressedKey!=27){
        pressedKey = getCharWithoutWaitingENTER();
        std::cout << "PressedKey: " << pressedKey << std::endl;
    }

    return NULL;
}

void* mainThreadFunction(void* arg)
{
    ros::NodeHandle n("~");
    ros::Rate rate(5); // run 5 times per second
    
    Action action;
    Perception perception;

    // Initialize publishers and subscribers
    ros::Publisher  pub_twist = n.advertise<geometry_msgs::Twist>("/rosaria_phi/cmd_vel", 1);
    ros::Subscriber sub_laser = n.subscribe("/rosaria_phi/laser_laserscan", 100, &Perception::receiveLaser, &perception);
    ros::Subscriber sub_sonar = n.subscribe("/rosaria_phi/sonar", 100, &Perception::receiveSonar, &perception);
    
    while(pressedKey!=27){

        // Get latest sensor readings
        std::vector<float> lasers = perception.getLatestLaserRanges();
        std::vector<float> sonars = perception.getLatestSonarRanges();
        std::cout << "Read " << lasers.size() << " laser measurements" << std::endl;
        std::cout << "Read " << sonars.size() << " sonar measurements" << std::endl; 
        
        // Get keyboard input
        char ch = pressedKey; 
        MotionControl mc = action.handlePressedKey(ch);
        std::cout << mc.mode << ' ' << mc.direction << std::endl;

        // Compute next action
        if(mc.mode == MANUAL){
            action.manualRobotMotion(mc.direction);
        }else if(mc.mode == WANDER){
            action.avoidObstacles(lasers,sonars);
        }else if(mc.mode == WALLFOLLOW){
            action.wallFollowing(lasers,sonars);
        }
        
        action.correctVelocitiesIfInvalid();
        
        // Publish robot motion
        geometry_msgs::Twist twistROS;
        twistROS.linear.x = action.getLinearVelocity();
        twistROS.angular.z = action.getAngularVelocity();
        pub_twist.publish(twistROS);
        std::cout << "Published linVel " << twistROS.linear.x << " angVel " << twistROS.angular.z << std::endl;

        ros::spinOnce();
        rate.sleep();
    
    }
    
    return NULL;
}


int main(int argc, char** argv)
{
    pressedKey='x';

    ros::init(argc, argv, "control_P3DX");
    ROS_INFO("control_P3DX!");
    
    pthread_t mainThread, keyboardThread;

    pthread_create(&(mainThread),NULL,mainThreadFunction,NULL);
    pthread_create(&(keyboardThread),NULL,keyboardThreadFunction,NULL);

    pthread_join(mainThread, 0);
    pthread_join(keyboardThread, 0);
    
    return 0;
}
