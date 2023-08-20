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
    Perception perception(n);

    // Initialize publishers and subscribers
    ros::Publisher  pub_twist = n.advertise<geometry_msgs::Twist>("/rosaria_phi/cmd_vel", 1);
    
    while(pressedKey!=27){

         // Get keyboard input
        char ch = pressedKey; 
        MotionControl mc = action.handlePressedKey(ch);
        std::cout << mc.mode << ' ' << mc.direction << std::endl;

        // Compute next action
        if(mc.mode == MANUAL){
            action.manualRobotMotion(mc.direction);
        }else if(mc.mode == EXPLORE){
            if(perception.hasValidDirection()){
                double angle = perception.getDirectionOfNavigation();
                action.followDirection(angle);
            }else{
                action.stopRobot();   
            }
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

    ros::init(argc, argv, "exploration");
    ROS_INFO("exploration");
    
    pthread_t mainThread, keyboardThread;

    pthread_create(&(mainThread),NULL,mainThreadFunction,NULL);
    pthread_create(&(keyboardThread),NULL,keyboardThreadFunction,NULL);

    pthread_join(mainThread, 0);
    pthread_join(keyboardThread, 0);
    
    return 0;
}
