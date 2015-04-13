#include "ConstrainedHierarchicalIK.h"

using namespace VirtualRobot;

ConstrainedHierarchicalIK::ConstrainedHierarchicalIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet, float stepSize, int maxIterations, float stall_epsilon) :
    ConstrainedIK(robot, maxIterations, stall_epsilon),
    nodeSet(nodeSet),
    stepSize(stepSize)
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
    THROW_VR_EXCEPTION_IF(!ik, "IK not initialized, did you forget to call initialize()?");

    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians, stepSize);

    // Check the stall condition
    if(lastDelta.rows() > 0 && (delta - lastDelta).norm() < stallEpsilon)
    {
        VR_INFO << "Constrained IK failed due to stall condition" << std::endl;
        return false;
    }
    lastDelta = delta;

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
