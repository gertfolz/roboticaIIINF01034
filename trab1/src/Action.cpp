#include "Action.h"

#include <cmath>
#include <iostream>
#include <unistd.h>

Action::Action()
{
    linVel = 0.0;
    angVel = 0.0;
    desiredDistance = 0.9;
    Kp = 0.1;
    Ki = 0.001;
    Kd = 3;
    PDIIntegral = 0.0;
    PDICte = 0.0;
    dp_params[0] = 0.3;
    dp_params[1] = 0.003;
    dp_params[2] = 3;
    params[0] = Kp;
    params[1] = Ki;
    params[2] = Kd;
}

void Action::avoidObstacles(std::vector<float> lasers, std::vector<float> sonars)
{
    bool obstacle_front = searchObstacles(lasers, sonars, 70, 110, 1.5);
    bool obstacle_left = searchObstacles(lasers, sonars, 10, 30, 1);
    bool obstacle_right = searchObstacles(lasers, sonars, 150, 170, 1);
    bool turning_right = false;
    bool turning_left = false;

    if (obstacle_front)
    {
        linVel = 0.0;
        if (!obstacle_left && !turning_right)
        {
            turning_left = true;
            angVel = 0.3;
        }

        else if (!turning_left)
        {
            turning_right = true;
            angVel = -0.3;
        }
    }
    else
    {
        turning_right = false;
        turning_left = false;
        linVel = 0.5;
        angVel = 0.0;
    }
}

void Action::wallFollowing(std::vector<float> lasers, std::vector<float> sonars)
{

    double CTE = computeCTE(lasers, desiredDistance);
    PDIIntegral += CTE;
    double deltaError = CTE - PDICte;
    double proportional = Kp * CTE;
    double integralTerm = Ki * PDIIntegral;
    double derivative = Kd * deltaError;
    double totalerr = -(proportional + integralTerm + derivative);
    angVel = -totalerr;
    if (angVel > 0.5)
        angVel = 0.5;
    if (angVel < -0.5)
        angVel = -0.5;
    twiddle(CTE * CTE);
    Kp = params[0];
    Ki = params[1];
    Kd = params[2];
    printParams();
    bool obstacle_front = searchObstacles(lasers, sonars, 70, 110, 0.75);
    if (obstacle_front)
        linVel = 0.0;
    else 
        linVel = 0.4;
    PDICte = CTE;
}

void Action::manualRobotMotion(MovingDirection direction)
{
    if (direction == FRONT)
    {
        linVel = 0.5;
        angVel = 0.0;
    }
    else if (direction == BACK)
    {
        linVel = -0.5;
        angVel = 0.0;
    }
    else if (direction == LEFT)
    {
        linVel = 0.0;
        angVel = 0.5;
    }
    else if (direction == RIGHT)
    {
        linVel = 0.0;
        angVel = -0.5;
    }
    else if (direction == STOP)
    {
        linVel = 0.0;
        angVel = 0.0;
    }
}

void Action::correctVelocitiesIfInvalid()
{
    float b = 0.38;

    float leftVel = linVel - angVel * b / (2.0);
    float rightVel = linVel + angVel * b / (2.0);

    float VELMAX = 0.5;

    float absLeft = fabs(leftVel);
    float absRight = fabs(rightVel);

    if (absLeft > absRight)
    {
        if (absLeft > VELMAX)
        {
            leftVel *= VELMAX / absLeft;
            rightVel *= VELMAX / absLeft;
        }
    }
    else
    {
        if (absRight > VELMAX)
        {
            leftVel *= VELMAX / absRight;
            rightVel *= VELMAX / absRight;
        }
    }

    linVel = (leftVel + rightVel) / 2.0;
    angVel = (rightVel - leftVel) / b;
}

float Action::getLinearVelocity()
{
    return linVel;
}

float Action::getAngularVelocity()
{
    return angVel;
}
// TODO: COMPLETAR FUNCAO
MotionControl Action::handlePressedKey(char key)
{
    MotionControl mc;
    mc.mode = MANUAL;
    mc.direction = STOP;

    if (key == '1')
    {
        mc.mode = MANUAL;
        mc.direction = STOP;
    }
    else if (key == '2')
    {
        mc.mode = WANDER;
        mc.direction = AUTO;
    }
    else if (key == '3')
    {
        mc.mode = WALLFOLLOW;
        mc.direction = AUTO;
    }
    else if (key == 'w' or key == 'W')
    {
        mc.mode = MANUAL;
        mc.direction = FRONT;
    }
    else if (key == 's' or key == 'S')
    {
        mc.mode = MANUAL;
        mc.direction = BACK;
    }
    else if (key == 'a' or key == 'A')
    {
        mc.mode = MANUAL;
        mc.direction = LEFT;
    }
    else if (key == 'd' or key == 'D')
    {
        mc.mode = MANUAL;
        mc.direction = RIGHT;
    }
    else if (key == ' ')
    {
        mc.mode = MANUAL;
        mc.direction = STOP;
    }

    return mc;
}

float Action::measureSensorsDiff(std::vector<float> lasers, std::vector<float> sonars)
{
    float leftLasersAverageDistance = (lasers[0] + lasers[1] + lasers[2]) / 3;
    float diagonalLeftLasersAverageDistance = (lasers[44] + lasers[45] + lasers[46]) / 3;
    return diagonalLeftLasersAverageDistance * cos(45 * 3.14159 / 180) - leftLasersAverageDistance;
}

bool Action::searchObstacles(std::vector<float> lasers, std::vector<float> sonars, int begin, int end, float distance)
{
    bool obstacle = false;
    for (int i = begin; i < end; i++)
    {
        if (lasers[i] < distance)
            obstacle = true;
    }
    return obstacle;
}

double Action::computeCTE(std::vector<float> lasers, float desiredDistance)
{
    double leftLasersMinDistance = 100;
    for (int i = 0; i < 90; i++)
    {
        if (lasers[i] < leftLasersMinDistance)
            leftLasersMinDistance = lasers[i];
    }
    double CTE = leftLasersMinDistance - desiredDistance;

    if (CTE > 1.0)
        CTE = 1.0;
    else if (CTE < -1.0)
        CTE = -1.0;
    std::cout << "CTE: " << CTE << "\n";
    return CTE;
}

void Action::twiddle(double CTE)
{
    float sum_p = 0.0;
    for (int i = 0; i<3; i++)
    {
        sum_p += dp_params[i];
    }
    if (sum_p < 0.00002)
    {
        return;
    }
    if (!st1_flag)
    {
        params[idx_opt] += dp_params[idx_opt];
        st1_flag = true;
        return;
    }
    if (CTE < best_error && !part2)
    {
        best_error = CTE;

        dp_params[idx_opt] *= 1.1;
        st1_flag = false;
        idx_opt = idx_opt + 1;

        if (idx_opt > 2)
        {
            idx_opt = 0;
            resetError();
        }
    }
    else
    {
        if (!st2_flag)
        {
            params[idx_opt] -= 2 * dp_params[idx_opt];
            st2_flag = true;
            part2 = true;
            return;
        }
        if (CTE < best_error)
        {
            best_error = CTE;
            dp_params[idx_opt] *= 1.1;
        }
        else
        {
            params[idx_opt] += dp_params[idx_opt];
            dp_params[idx_opt] *= 0.9;
        }

        st1_flag = false;
        st2_flag = false;
        part2 = false;
        idx_opt = idx_opt + 1;
        if (idx_opt > 2)
        {
            idx_opt = 0;
            resetError();
        }
    }
}

void Action::printParams()
{
    std::cout << "***Parameters";
    std::cout << "\nKp: " << params[0] << "\nKi: " << params[1] << "\nKd: " << params[2];
}

void Action::resetError(){
    PDIIntegral = 0;
    cur_error = 0;
}

