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
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "PositionConstraint.h"

#include <VirtualRobot/Robot.h>

using namespace VirtualRobot;

PositionConstraint::PositionConstraint(const VirtualRobot::RobotPtr &robot, const VirtualRobot::RobotNodeSetPtr &nodeSet, const VirtualRobot::SceneObjectPtr &eef,
                                                     const Eigen::Vector3f &target, VirtualRobot::IKSolver::CartesianSelection cartesianSelection, float tolerance) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    target(target),
    cartesianSelection(cartesianSelection),
    tolerance(tolerance)
{
    ik.reset(new DifferentialIK(nodeSet));
    Eigen::Matrix4f target4x4 = Eigen::Matrix4f::Identity();
    target4x4.block<3,1>(0,3) = target;
    ik->setGoal(target, eef, cartesianSelection);
    addOptimizationFunction(0, false);
}

double PositionConstraint::optimizationFunction(unsigned int /*id*/)
{
    Eigen::Vector3f d = eef->getGlobalPose().block<3,1>(0,3) - target;

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            return optimizationFunctionFactor * d.x() * d.x();

        case IKSolver::CartesianSelection::Y:
            return optimizationFunctionFactor * d.y() * d.y();

        case IKSolver::CartesianSelection::Z:
            return optimizationFunctionFactor * d.z() * d.z();

        case IKSolver::CartesianSelection::Position:
        case IKSolver::CartesianSelection::All:
            return optimizationFunctionFactor * d.dot(d);

        case IKSolver::CartesianSelection::Orientation:
            return 0;
    }

    return 0;
}

Eigen::VectorXf PositionConstraint::optimizationGradient(unsigned int /*id*/)
{
    int size = nodeSet->getSize();

    Eigen::MatrixXf J = ik->getJacobianMatrix(eef).block(0, 0, 3, size);
    Eigen::Vector3f d = eef->getGlobalPose().block<3,1>(0,3) - target;

    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            return 2 * optimizationFunctionFactor * Eigen::Vector3f(d.x(), 0, 0).transpose() * J;

        case IKSolver::CartesianSelection::Y:
            return 2 * optimizationFunctionFactor * Eigen::Vector3f(0, d.y(), 0).transpose() * J;

        case IKSolver::CartesianSelection::Z:
            return 2 * optimizationFunctionFactor * Eigen::Vector3f(0, 0, d.z()).transpose() * J;

        case IKSolver::CartesianSelection::Position:
        case IKSolver::CartesianSelection::All:
            return 2 * optimizationFunctionFactor * d.transpose() * J;

        case IKSolver::CartesianSelection::Orientation:
            return Eigen::VectorXf::Zero(size);
    }

    return Eigen::VectorXf::Zero(size);
}

bool PositionConstraint::checkTolerances()
{
    Eigen::Vector3f d = eef->getGlobalPose().block<3,1>(0,3) - target;
    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
            return d.x() <= tolerance;

        case IKSolver::CartesianSelection::Y:
            return d.y() <= tolerance;

        case IKSolver::CartesianSelection::Z:
            return d.z() <= tolerance;

        case IKSolver::CartesianSelection::Position:
        case IKSolver::CartesianSelection::All:
            return d.norm() <= tolerance;

        case IKSolver::CartesianSelection::Orientation:
            return true;
    }
    std::stringstream ss;
    ss << "PositionConstraint::checkTolerances(): unknown value for cartesianSelection = " << cartesianSelection;
    throw std::logic_error{ss.str()};
}
