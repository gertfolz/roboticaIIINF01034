#include <ros/ros.h>

#include "Perception.h"

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

    Perception perception(n);

    Pose2D previousOdom;

    while(pressedKey!=27)
    {
        if(perception.hasReceivedMap())
        {
            if(perception.hasStartedMCL() == false)
            {
                perception.MCL_initialize();
            }
            else
            {
                Pose2D currentOdom = perception.getCurrentRobotPose();
               
                Action u;
                u.rot1 = atan2(currentOdom.y-previousOdom.y,currentOdom.x-previousOdom.x)-DEG2RAD(currentOdom.theta);
                u.trans = sqrt(pow(currentOdom.x-previousOdom.x,2)+pow(currentOdom.y-previousOdom.y,2));
                u.rot2 = DEG2RAD(currentOdom.theta)-DEG2RAD(previousOdom.theta)-u.rot1;

                // check if there is enough robot motion
                if(u.trans > 0.1 || fabs(normalizeAngleDEG(currentOdom.theta-previousOdom.theta)) > 30)
                {
                    perception.MCL_sampling(u);
                    perception.MCL_weighting(perception.getLaserReadings());
                    perception.MCL_resampling();

                    previousOdom = currentOdom;
                }
            }

            perception.MCL_publishParticles();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


int main(int argc, char** argv)
{
    pressedKey='x';

    ros::init(argc, argv, "localization");
    ROS_INFO("localization");
    
    pthread_t mainThread, keyboardThread;

    pthread_create(&(mainThread),NULL,mainThreadFunction,NULL);
    pthread_create(&(keyboardThread),NULL,keyboardThreadFunction,NULL);

    pthread_join(mainThread, 0);
    pthread_join(keyboardThread, 0);
    
    return 0;
}