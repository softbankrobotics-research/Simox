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

#include <stdexcept>
#include <sstream>

#include "OrientationConstraint.h"

#include <VirtualRobot/Model/Model.h>

using namespace VirtualRobot;

OrientationConstraint::OrientationConstraint(const VirtualRobot::RobotPtr &robot, const VirtualRobot::RobotNodeSetPtr &nodeSet, const VirtualRobot::SceneObjectPtr &eef,
                                                           const Eigen::Matrix3f &target, VirtualRobot::IKSolver::CartesianSelection cartesianSelection, float tolerance, bool soft) :
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
    target4x4.block<3,3>(0,0) = target;
    ik->setGoal(target4x4, eef, cartesianSelection);
    addOptimizationFunction(0, soft);
}

double OrientationConstraint::optimizationFunction(unsigned int /*id*/)
{
    Eigen::Matrix3f diff = target * eef->getGlobalPose().block<3,3>(0,0).inverse();
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
            value = optimizationFunctionFactor * aa.angle() * aa.angle();
            break;
    }

    return value;
}

Eigen::VectorXf OrientationConstraint::optimizationGradient(unsigned int /*id*/)
{
    int size = nodeSet->getSize();

    Eigen::MatrixXf J = ik->getJacobianMatrix(eef);
    Eigen::Matrix3f diff = eef->getGlobalPose().block<3,3>(0,0) * target.inverse();

    Eigen::Vector3f rpy;
    Eigen::AngleAxisf aa;
    aa = diff;
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
            return 2 * optimizationFunctionFactor * rpy.transpose() * J.block(3, 0, 3, size);
    }

    return Eigen::VectorXf::Zero(size);
}

bool OrientationConstraint::checkTolerances()
{
    switch(cartesianSelection)
    {
        case IKSolver::CartesianSelection::X:
        case IKSolver::CartesianSelection::Y:
        case IKSolver::CartesianSelection::Z:
        case IKSolver::CartesianSelection::Position:
            return true;
            break;

        case IKSolver::CartesianSelection::Orientation:
        case IKSolver::CartesianSelection::All:
        {
            Eigen::Matrix3f diff = target * eef->getGlobalPose().block<3,3>(0,0).inverse();
            Eigen::AngleAxisf aa(diff);

            return fabs(aa.angle()) <= tolerance;
        }
            break;
    }
    std::stringstream ss;
    ss << "OrientationConstraint::checkTolerances(): unknown value for cartesianSelection = " << cartesianSelection;
    throw std::logic_error{ss.str()};
}
