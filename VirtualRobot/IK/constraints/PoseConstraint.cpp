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
    toleranceRotation(toleranceRotation)
{
    ik.reset(new DifferentialIK(nodeSet));
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);
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

double PoseConstraint::optimizationFunction()
{
    Eigen::Vector3f rpy;
    float tradeoff = 100;

    Eigen::Vector3f d = eef->getGlobalPose().block<3,1>(0,3) - target.block<3,1>(0,3);
    Eigen::Matrix4f diff = eef->getGlobalPose() * target.inverse();
    MathTools::eigen4f2rpy(diff, rpy);

    float value = 0;
    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            value = d.x() * d.x();
            break;

        case IKSolver::CartesianSelection::Y:
            value = d.y() * d.y();
            break;

        case IKSolver::CartesianSelection::Z:
            value = d.z() * d.z();
            break;

        case IKSolver::CartesianSelection::Position:
            value = d.dot(d);
            break;

        case IKSolver::CartesianSelection::Orientation:
            value = rpy.dot(rpy);
            break;

        case IKSolver::CartesianSelection::All:
            value = d.dot(d) + tradeoff * tradeoff * rpy.dot(rpy);
            break;
    }

    return value;
}

Eigen::VectorXf PoseConstraint::optimizationGradient()
{
    float tradeoff = 100;
    int size = nodeSet->getSize();

    Eigen::VectorXf g(Eigen::VectorXf::Zero(size));
    Eigen::MatrixXf J = ik->getJacobianMatrix(eef);

    Eigen::Vector3f d = eef->getGlobalPose().block<3,1>(0,3) - target.block<3,1>(0,3);
    Eigen::Matrix4f diff = eef->getGlobalPose() * target.inverse();

    Eigen::Vector3f rpy;
    MathTools::eigen4f2rpy(diff, rpy);

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            g += 2 * Eigen::Vector3f(d.dot(Eigen::Vector3f::UnitX())).transpose() * J.block(0, 0, 3, size);
            break;

        case IKSolver::CartesianSelection::Y:
            g += 2 * Eigen::Vector3f(d.dot(Eigen::Vector3f::UnitY())).transpose() * J.block(0, 0, 3, size);
            break;

        case IKSolver::CartesianSelection::Z:
            g += 2 * Eigen::Vector3f(d.dot(Eigen::Vector3f::UnitZ())).transpose() * J.block(0, 0, 3, size);
            break;

        case IKSolver::CartesianSelection::Position:
            g += 2 * d.transpose() * J.block(0, 0, 3, size);
            break;

        case IKSolver::CartesianSelection::Orientation:
            g += 2 * rpy.transpose() * J.block(3, 0, 3, size);
            break;

        case IKSolver::CartesianSelection::All:
            g += 2 * d.transpose() * J.block(0, 0, 3, size) + 2 * tradeoff * tradeoff * rpy.transpose() * J.block(3, 0, 3, size);
            break;
    }

    return g / g.norm();
}

