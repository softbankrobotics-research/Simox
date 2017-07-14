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

#ifndef _VirtualRobot_CoMConstraint_h_
#define _VirtualRobot_CoMConstraint_h_

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/IK/CoMIK.h"


namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoMConstraint : public Constraint
    {
    public:
        CoMConstraint(const RobotPtr& robot, const JointSetPtr& joints, const LinkSetPtr& bodies, const Eigen::Vector3f& target, float tolerance);
        CoMConstraint(const RobotPtr& robot, const JointSetPtr& joints, const LinkSetPtr& bodies, const Eigen::Vector2f& target, float tolerance);

        double optimizationFunction(unsigned int id);
        Eigen::VectorXf optimizationGradient(unsigned int id);
        bool checkTolerances();

        void updateTarget(const Eigen::Vector3f& target);
        void updateTarget(const Eigen::Vector2f& target);

    protected:
        ModelPtr robot;
        JointSetPtr nodeSetJoints;
        LinkSetPtr nodeSetBodies;
        Eigen::VectorXf target;

        CoMIKPtr ik;
        int dimensions;
        float tolerance;
    };

    typedef std::shared_ptr<CoMConstraint> CoMConstraintPtr;
}

#endif

