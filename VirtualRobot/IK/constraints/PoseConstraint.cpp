/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "PoseConstraint.h"

#include <VirtualRobot/Robot.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>

#include <iostream>

using namespace VirtualRobot;

#define POSITION_COMPONENT      0
#define ORIENTATION_COMPONENT   1

PoseConstraint::PoseConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet, const SceneObjectPtr& eef, const Eigen::Matrix4f& target, IKSolver::CartesianSelection cartesianSelection,
                               float tolerancePosition, float toleranceRotation) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    target(target),
    cartesianSelection(cartesianSelection),
    tolerancePosition(tolerancePosition),
    toleranceRotation(toleranceRotation),
    posRotTradeoff(0.1f)
{
    ik.reset(new DifferentialIK(nodeSet));
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);

    addOptimizationFunction(POSITION_COMPONENT);
    addOptimizationFunction(ORIENTATION_COMPONENT);

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
        case POSITION_COMPONENT:
            return optimizationFunctionFactor * positionOptimizationFunction();

        case ORIENTATION_COMPONENT:
            return optimizationFunctionFactor * orientationOptimizationFunction();

        default:
            return 0;
    }
}

Eigen::VectorXf PoseConstraint::optimizationGradient(unsigned int id)
{
    switch(id)
    {
        case POSITION_COMPONENT:
            return optimizationFunctionFactor * positionOptimizationGradient();

        case ORIENTATION_COMPONENT:
            return optimizationFunctionFactor * orientationOptimizationGradient();

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
    return 0;
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
    return Eigen::VectorXf::Zero(size);
}

double PoseConstraint::orientationOptimizationFunction()
{
    Eigen::Matrix3f diff = target.block<3,3>(0,0) * eef->getGlobalPose().block<3,3>(0,0).inverse();
    Eigen::AngleAxisf aa(diff);

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
            value = aa.angle() * aa.angle();
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
    return Eigen::VectorXf::Zero(size);
}
