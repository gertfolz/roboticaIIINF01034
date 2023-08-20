#ifndef ACTION_H
#define ACTION_H

enum MotionMode {MANUAL, EXPLORE};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(MovingDirection direction);
    void followDirection(double angle);
    void stopRobot();

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:


    float linVel;
    float angVel;
};

#endif // ACTION_H
