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
        HierarchicalIK::JacobiDefinition jac;
        jac.jacProvider = constraint;
        jacobians.push_back(jac);
    }

    ConstrainedIK::initialize();
}

bool ConstrainedHierarchicalIK::solveStep()
{
    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians);

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
