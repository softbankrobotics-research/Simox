#include "Constraint.h"

using namespace VirtualRobot;

Constraint::Constraint(const RobotNodeSetPtr& nodeSet) :
    JacobiProvider(nodeSet, JacobiProvider::eSVDDamped),
    lastError(-1),
    lastLastError(-1)
{
}

void Constraint::initialize()
{
    lastError = -1;
    lastLastError = -1;
}

bool Constraint::getRobotPoseForConstraint(Eigen::Matrix4f& pose)
{
    // No change in global pose required
    return false;
}

float Constraint::getErrorDifference()
{
    Eigen::VectorXf e = getError(1);
    lastLastError = lastError;
    lastError = e.norm();

    if (lastLastError <= 0)
    {
        return 0;
    }

    return lastLastError - lastError;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getEqualityConstraints()
{
    return equalityConstraints;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getInequalityConstraints()
{
    return inequalityConstraints;
}

const std::vector<OptimizationFunctionSetup> &Constraint::getOptimizationFunctions()
{
    return optimizationFunctions;
}

void Constraint::addEqualityConstraint(unsigned int id)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    equalityConstraints.push_back(setup);
}

void Constraint::addInequalityConstraint(unsigned int id)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    inequalityConstraints.push_back(setup);
}

void Constraint::addOptimizationFunction(unsigned int id)
{
    OptimizationFunctionSetup setup;
    setup.id = id;
    setup.constraint = this;
    optimizationFunctions.push_back(setup);
}
