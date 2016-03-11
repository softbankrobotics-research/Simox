#include "PoseConstraint.h"

#include <VirtualRobot/Robot.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>

#include <iostream>

using namespace VirtualRobot;

PoseConstraint::PoseConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet, const RobotNodePtr& eef, const Eigen::Matrix4f& target, IKSolver::CartesianSelection cartesianSelection,
                               float tolerancePosition, float toleranceRotation) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    target(target),
    cartesianSelection(cartesianSelection),
    tolerancePosition(tolerancePosition),
    toleranceRotation(toleranceRotation),
    posRotTradeoff(0.1)
{
    ik.reset(new DifferentialIK(nodeSet));
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);

    addOptimizationFunction(0);
    addOptimizationFunction(1);

    initialized = true;
}

void PoseConstraint::setVisualization(const SceneObjectSetPtr& visualizationNodeSet)
{
    this->visualizationNodeSet = visualizationNodeSet;
}

Eigen::MatrixXf PoseConstraint::getJacobianMatrix()
{
    return ik->getJacobianMatrix(eef, cartesianSelection);
}

Eigen::MatrixXf PoseConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    if (tcp->getName() != eef->getName())
    {
        VR_WARNING << "EndEffectorPoseConstraing Jacobina calculation for differing EEF ('" << tcp->getName() << "' instead of '" << eef->getName() << "')" << std::endl;
    }

    return ik->getJacobianMatrix(tcp);
}

Eigen::VectorXf PoseConstraint::getError(float stepSize)
{
    return ik->getError(stepSize);
}

bool PoseConstraint::checkTolerances()
{
    return ik->checkTolerances();
}

bool PoseConstraint::getRobotPoseForConstraint(Eigen::Matrix4f& pose)
{
    if (robot->getRootNode()->getName() == eef->getName())
    {
        // If the end effector to move equals the robot root, we initially move the whole robot to this
        // position in order to satisfy this constraint
        pose = target;
        return true;
    }

    return false;
}

std::string PoseConstraint::getConstraintType()
{
    return "Pose(" + eef->getName() + ")";
}

const Eigen::Matrix4f& PoseConstraint::getTarget()
{
    return target;
}

void PoseConstraint::updateTarget(const Eigen::Matrix4f& newTarget)
{
    target = newTarget;
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);
}

double PoseConstraint::optimizationFunction(unsigned int id)
{
    switch(id)
    {
        case 0:
            return positionOptimizationFunction();

        case 1:
            return orientationOptimizationFunction();

        default:
            return 0;
    }
}

Eigen::VectorXf PoseConstraint::optimizationGradient(unsigned int id)
{
    switch(id)
    {
        case 0:
            return positionOptimizationGradient();

        case 1:
            return orientationOptimizationGradient();

        default:
            return Eigen::VectorXf();
    }
}

double PoseConstraint::positionOptimizationFunction()
{
    Eigen::Vector3f d = posRotTradeoff * (eef->getGlobalPose().block<3,1>(0,3) - target.block<3,1>(0,3));

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            return d.x() * d.x();

        case IKSolver::CartesianSelection::Y:
            return d.y() * d.y();

        case IKSolver::CartesianSelection::Z:
            return d.z() * d.z();

        case IKSolver::CartesianSelection::Position:
        case IKSolver::CartesianSelection::All:
            return d.dot(d);

        case IKSolver::CartesianSelection::Orientation:
            return 0;
    }
}

Eigen::VectorXf PoseConstraint::positionOptimizationGradient()
{
    int size = nodeSet->getSize();

    Eigen::MatrixXf J = ik->getJacobianMatrix(eef).block(0, 0, 3, size);
    Eigen::Vector3f d = posRotTradeoff * (eef->getGlobalPose().block<3,1>(0,3) - target.block<3,1>(0,3));

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            return 2 * Eigen::Vector3f(d.x(), 0, 0).transpose() * J;

        case IKSolver::CartesianSelection::Y:
            return 2 * Eigen::Vector3f(0, d.y(), 0).transpose() * J;

        case IKSolver::CartesianSelection::Z:
            return 2 * Eigen::Vector3f(0, 0, d.z()).transpose() * J;

        case IKSolver::CartesianSelection::Position:
        case IKSolver::CartesianSelection::All:
            return 2 * d.transpose() * J;

        case IKSolver::CartesianSelection::Orientation:
            return Eigen::VectorXf::Zero(size);
    }
}

double PoseConstraint::orientationOptimizationFunction()
{
    Eigen::Vector3f rpy;
    Eigen::Matrix4f diff = eef->getGlobalPose() * target.inverse();

    Eigen::AngleAxisf aa;
    aa = diff.block<3,3>(0,0);
    rpy = aa.angle() * aa.axis();

    float value = 0;
    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
        case IKSolver::CartesianSelection::Y:
        case IKSolver::CartesianSelection::Z:
        case IKSolver::CartesianSelection::Position:
            break;

        case IKSolver::CartesianSelection::Orientation:
        case IKSolver::CartesianSelection::All:
            value = rpy.dot(rpy);
            break;
    }

    return value;
}

Eigen::VectorXf PoseConstraint::orientationOptimizationGradient()
{
    int size = nodeSet->getSize();

    Eigen::MatrixXf J = ik->getJacobianMatrix(eef);
    Eigen::Matrix4f diff = eef->getGlobalPose() * target.inverse();

    Eigen::Vector3f rpy;
    Eigen::AngleAxisf aa;
    aa = diff.block<3,3>(0,0);
    rpy = aa.angle() * aa.axis();

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
        case IKSolver::CartesianSelection::Y:
        case IKSolver::CartesianSelection::Z:
        case IKSolver::CartesianSelection::Position:
            return Eigen::VectorXf::Zero(size);

        case IKSolver::CartesianSelection::Orientation:
        case IKSolver::CartesianSelection::All:
            return 2 * rpy.transpose() * J.block(3, 0, 3, size);
    }
}
