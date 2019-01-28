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

#include "CoMConstraint.h"

#include <VirtualRobot/Robot.h>

using namespace VirtualRobot;

CoMConstraint::CoMConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const Eigen::Vector3f &target, float tolerance) :
    Constraint(joints),
    robot(robot),
    nodeSetJoints(joints),
    nodeSetBodies(bodies),
    target(target),
    dimensions(3),
    tolerance(tolerance)
{
    ik.reset(new CoMIK(nodeSetJoints, nodeSetBodies, RobotNodePtr(), 3));
    ik->setGoal(target);

    addOptimizationFunction(0, false);
}

CoMConstraint::CoMConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const Eigen::Vector2f &target, float tolerance) :
    Constraint(joints),
    robot(robot),
    nodeSetJoints(joints),
    nodeSetBodies(bodies),
    target(target),
    dimensions(2),
    tolerance(tolerance)
{
    ik.reset(new CoMIK(nodeSetJoints, nodeSetBodies));
    ik->setGoal(target);

    addOptimizationFunction(0, false);
}

void CoMConstraint::updateTarget(const Eigen::Vector3f &target)
{
    ik->setGoal(target);
}

void CoMConstraint::updateTarget(const Eigen::Vector2f &target)
{
    ik->setGoal(target);
}

double CoMConstraint::optimizationFunction(unsigned int /*id*/)
{
    if(dimensions == 2)
    {
        Eigen::Vector2f d = nodeSetBodies->getCoM().head(2) - target;
        return optimizationFunctionFactor * d.dot(d);
    }
    else if(dimensions == 3)
    {
        Eigen::Vector3f d = nodeSetBodies->getCoM() - target;
        return optimizationFunctionFactor * d.dot(d);
    }

    return 0;
}

Eigen::VectorXf CoMConstraint::optimizationGradient(unsigned int /*id*/)
{
    int size = nodeSetJoints->getSize();

    if(dimensions == 2)
    {
        Eigen::MatrixXf J = ik->getJacobianMatrix().block(0, 0, 2, size);
        Eigen::Vector2f d = nodeSetBodies->getCoM().head(2) - target;
        return 2 * optimizationFunctionFactor * d.transpose() * J;
    }
    else if(dimensions == 3)
    {
        Eigen::MatrixXf J = ik->getJacobianMatrix();
        Eigen::Vector3f d = nodeSetBodies->getCoM() - target;
        return 2 * optimizationFunctionFactor * d.transpose() * J;
    }

    return Eigen::VectorXf::Zero(size);
}

bool CoMConstraint::checkTolerances()
{
    if(dimensions == 2)
    {
        Eigen::Vector2f d = nodeSetBodies->getCoM().head(2) - target;
        return d.norm() <= tolerance;
    }
    else if(dimensions == 3)
    {
        Eigen::Vector3f d = nodeSetBodies->getCoM() - target;
        return d.norm() <= tolerance;
    }

    return false;
}
