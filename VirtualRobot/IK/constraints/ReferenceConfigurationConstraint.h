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

#pragma once

#include "VirtualRobot/IK/Constraint.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ReferenceConfigurationConstraint : public Constraint
    {
    public:
        ReferenceConfigurationConstraint(const ModelPtr& robot, const JointSetPtr& nodeSet);
        ReferenceConfigurationConstraint(const ModelPtr& robot, const JointSetPtr& nodeSet, const Eigen::VectorXf &reference);

        void setReferenceConfiguration(const Eigen::VectorXf &config);
        Eigen::VectorXf getReferenceConfiguration();

        double optimizationFunction(unsigned int id) override;
        Eigen::VectorXf optimizationGradient(unsigned int id) override;

    protected:
        ModelPtr robot;
        JointSetPtr nodeSet;
        Eigen::VectorXf reference;
    };

    typedef std::shared_ptr<ReferenceConfigurationConstraint> ReferenceConfigurationConstraintPtr;
}

