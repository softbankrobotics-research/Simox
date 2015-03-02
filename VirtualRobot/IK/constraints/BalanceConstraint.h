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
#ifndef _VirtualRobot_BalanceConstraint_h_
#define _VirtualRobot_BalanceConstraint_h_

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/Constraint.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <VirtualRobot/IK/SupportPolygon.h>

#include <boost/shared_ptr.hpp>

class SoSeparator;
class SoNode;

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT BalanceConstraint : public Constraint, public boost::enable_shared_from_this<BalanceConstraint>
    {
        public:
            BalanceConstraint(const RobotPtr &robot,
                              const RobotNodeSetPtr &joints,
                              const RobotNodeSetPtr &bodies,
                              const SceneObjectSetPtr &contactNodes,
                              float tolerance, float minimumStability);

            Eigen::MatrixXf getJacobianMatrix();
            Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp);
            Eigen::VectorXf getError(float stepSize);
            bool checkTolerances();

            bool getRobotPoseForConstraint(RobotPtr &robot, Eigen::Matrix4f &pose);

            void visualize(SoSeparator *sep);
            void visualizeContinuously(SoSeparator *sep);

            std::string getConstraintType();

        protected:
            CoMIKPtr comIK;
            SupportPolygonPtr supportPolygon;

            RobotNodeSetPtr joints;
            RobotNodeSetPtr bodies;

            float minimumStability;
    };

    typedef boost::shared_ptr<BalanceConstraint> BalanceConstraintPtr;
}

#endif