#include "ConstrainedHierarchicalIK.h"

using namespace VirtualRobot;

ConstrainedHierarchicalIK::ConstrainedHierarchicalIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet) :
    ConstrainedIK(robot),
    nodeSet(nodeSet)
{
}

bool ConstrainedHierarchicalIK::initialize()
{
    ik.reset(new HierarchicalIK(nodeSet));
    jacobians.clear();

    for(auto &constraint : constraints)
    {
        jacobians.push_back(constraint);
    }

    return ConstrainedIK::initialize();
}

bool ConstrainedHierarchicalIK::solveStep()
{
    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians);

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
