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

#ifndef _VirtualRobot_PositionConstraint_h_
#define _VirtualRobot_PositionConstraint_h_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/IK/DifferentialIK.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT PositionConstraint : public Constraint
    {
    public:
        PositionConstraint(const ModelPtr& robot, const JointSetPtr& nodeSet, const FramePtr& eef, const Eigen::Vector3f& target,
                       IKSolver::CartesianSelection cartesianSelection = IKSolver::All, float tolerance = 3.0f);

        virtual double optimizationFunction(unsigned int id) override;
        virtual Eigen::VectorXf optimizationGradient(unsigned int id) override;
        virtual bool checkTolerances() override;

        Eigen::Vector3f getTarget();

    protected:
        RobotPtr robot;
        JointSetPtr nodeSet;
        FramePtr eef;
        Eigen::Vector3f target;

        DifferentialIKPtr ik;
        IKSolver::CartesianSelection cartesianSelection;
        float tolerance;
    };

    typedef std::shared_ptr<PositionConstraint> PositionConstraintPtr;
}

#endif

