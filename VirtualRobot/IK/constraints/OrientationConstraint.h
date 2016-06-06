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

#ifndef _VirtualRobot_OrientationConstraint_h_
#define _VirtualRobot_OrientationConstraint_h_

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/IK/Constraint.h"
#include "VirtualRobot/IK/DifferentialIK.h"

#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT OrientationConstraint : public Constraint, public boost::enable_shared_from_this<OrientationConstraint>
    {
    public:
        OrientationConstraint(const RobotPtr& robot, const RobotNodeSetPtr& nodeSet, const SceneObjectPtr& eef, const Eigen::Matrix3f& target,
                       IKSolver::CartesianSelection cartesianSelection = IKSolver::All, float tolerance = 1.0f * M_PI / 180.0f);

        double optimizationFunction(unsigned int id);
        Eigen::VectorXf optimizationGradient(unsigned int id);
        bool checkTolerances();

    protected:
        RobotPtr robot;
        RobotNodeSetPtr nodeSet;
        SceneObjectPtr eef;
        Eigen::Matrix3f target;

        DifferentialIKPtr ik;
        IKSolver::CartesianSelection cartesianSelection;
        float tolerance;

    };

    typedef boost::shared_ptr<OrientationConstraint> OrientationConstraintPtr;
}

#endif


