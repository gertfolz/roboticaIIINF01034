#ifndef ACTION_H
#define ACTION_H

#include <vector>
#include <float.h>

enum MotionMode {MANUAL, WANDER, WALLFOLLOW};
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
    void avoidObstacles(std::vector<float> lasers, std::vector<float> sonars);
    void wallFollowing(std::vector<float> lasers, std::vector<float> sonars);
    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    float measureSensorsDiff(std::vector<float> lasers, std::vector<float> sonars);
    bool searchObstacles(std::vector<float> lasers, std::vector<float> sonars, int begin, int end, float distance);
    double computeCTE(std::vector<float> lasers, float desiredDistance);
    void twiddle(double CTE);
    void resetError();
    float linVel;
    float angVel;
    float PDIIntegral;
    float PDICte;
    float desiredDistance;
    void printParams();
    bool part2 {false};
    double cur_error{0.0};
    double params[3];
    double dp_params[3];
    int idx_opt{0};
    bool st1_flag{false};
    bool st2_flag{false};
    double best_error{DBL_MAX};

    double Kp;
    double Ki;
    double Kd;
};

#endif // ACTION_H
