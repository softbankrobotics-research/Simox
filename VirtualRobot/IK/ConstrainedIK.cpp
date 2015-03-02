#include "ConstrainedIK.h"

using namespace VirtualRobot;

ConstrainedIK::ConstrainedIK(RobotPtr &robot) :
    robot(robot),
    maxIterations(10000),
    currentIteration(0),
    running(false)
{

}

void ConstrainedIK::addConstraint(const ConstraintPtr &constraint)
{
    constraints.push_back(constraint);
    std::sort(constraints.begin(), constraints.end(), [](const ConstraintPtr& lhs, const ConstraintPtr& rhs){return lhs->getPriority() > rhs->getPriority();});
}

std::vector<ConstraintPtr> ConstrainedIK::getConstraints()
{
    return constraints;
}

bool ConstrainedIK::initialize()
{
    Eigen::Matrix4f P;
    int moves = 0;

    for(auto &constraint : constraints)
    {
        moves += constraint->getRobotPoseForConstraint(P);
    }

    if(moves == 1)
    {
        // Only one constraint requested to move the robot => No conflicts
        robot->setGlobalPose(P);
    }
    else if(moves > 1)
    {
        std::cout << "Error: Multiple constraints requested to move the robot" << std::endl;
        return false;
    }

    running = true;
    currentIteration = 0;
    return true;
}

bool ConstrainedIK::solve(bool stepwise)
{
    while(currentIteration < maxIterations)
    {
        if(!solveStep())
        {
            running = false;
            return false;
        }

        bool goalReached = true;
        for(auto &constraint : constraints)
        {
            if(!constraint->checkTolerances())
            {
                goalReached = false;
            }
        }

        if(goalReached)
        {
            running = false;
            return true;
        }

        currentIteration++;

        if(stepwise)
        {
            // Identify this case via running variable
            return false;
        }
    }

    running = false;
    return false;
}

void ConstrainedIK::setMaxIterations(int maxIterations)
{
    this->maxIterations = maxIterations;
}

int ConstrainedIK::getMaxIterations()
{
    return maxIterations;
}

bool ConstrainedIK::getRunning()
{
    return running;
}

int ConstrainedIK::getCurrentIteration()
{
    return currentIteration;
}
