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

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/IK/DifferentialIK.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT OrientationConstraint : public Constraint
    {
    public:
        OrientationConstraint(const ModelPtr& robot, const JointSetPtr& nodeSet, const FramePtr& eef, const Eigen::Matrix3f& target,
                       IKSolver::CartesianSelection cartesianSelection = IKSolver::All, float tolerance = 1.0f * M_PI / 180.0f, bool soft=false);

        double optimizationFunction(unsigned int id) override;
        Eigen::VectorXf optimizationGradient(unsigned int id) override;
        bool checkTolerances() override;

    protected:
        ModelPtr robot;
        JointSetPtr nodeSet;
        FramePtr eef;
        Eigen::Matrix3f target;

        DifferentialIKPtr ik;
        IKSolver::CartesianSelection cartesianSelection;
        float tolerance;

    };

    typedef std::shared_ptr<OrientationConstraint> OrientationConstraintPtr;
}



