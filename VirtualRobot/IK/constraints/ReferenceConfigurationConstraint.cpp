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

#include "ReferenceConfigurationConstraint.h"

using namespace VirtualRobot;

ReferenceConfigurationConstraint::ReferenceConfigurationConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet)
{
    // Use zero reference by default
    setReferenceConfiguration(Eigen::VectorXf::Zero(nodeSet->getSize()));

    // Joint limit avoidance is considered a soft constraint
    addOptimizationFunction(0, true);

    initialized = true;
}

ReferenceConfigurationConstraint::ReferenceConfigurationConstraint(const RobotPtr &robot, const RobotNodeSetPtr &nodeSet, const Eigen::VectorXf &reference) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    reference(reference)
{
    setReferenceConfiguration(reference);

    // Joint limit avoidance is considered a soft constraint
    addOptimizationFunction(0, true);

    initialized = true;
}

void ReferenceConfigurationConstraint::setReferenceConfiguration(const Eigen::VectorXf &config)
{
    if(nodeSet->getSize() != config.rows())
    {
        std::stringstream sstr;
        sstr << "Reference configuration does not match node set size: "  << " nodeSet size: " << nodeSet->getSize() << " reference joint set size: " << reference.rows();
        THROW_VR_EXCEPTION(sstr.str());
        return;
    }

    reference = config;
}

Eigen::VectorXf ReferenceConfigurationConstraint::getReferenceConfiguration()
{
    return reference;
}

double ReferenceConfigurationConstraint::optimizationFunction(unsigned int id)
{
    double value = 0;

    float v;
    for(size_t i = 0; i < nodeSet->getSize(); i++)
    {
        RobotNodePtr node = nodeSet->getNode(i);
        v = (node->getJointValue() - reference(i));
        value += v * v;
    }

    return optimizationFunctionFactor * value;
}

Eigen::VectorXf ReferenceConfigurationConstraint::optimizationGradient(unsigned int id)
{
    Eigen::VectorXf gradient(nodeSet->getSize());

    for(size_t i = 0; i < nodeSet->getSize(); i++)
    {
        gradient(i) = 2 * (nodeSet->getNode(i)->getJointValue() - reference(i));
    }

    return optimizationFunctionFactor * gradient;
}
