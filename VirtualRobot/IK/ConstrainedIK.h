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
#ifndef _VirtualRobot_ConstrainedIK_h_
#define _VirtualRobot_ConstrainedIK_h_

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/Constraint.h>
#include <VirtualRobot/Robot.h>

#include <vector>
#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedIK : public boost::enable_shared_from_this<ConstrainedIK>
    {
        public:
            ConstrainedIK(RobotPtr &robot);

            void addConstraint(const ConstraintPtr &constraint);
            std::vector<ConstraintPtr> getConstraints();

            virtual bool initialize();

            virtual bool solveStep() = 0;
            bool solve(bool stepwise = false);

            void setMaxIterations(int maxIterations);
            int getMaxIterations();

            bool getRunning();
            int getCurrentIteration();

        protected:
            std::vector<ConstraintPtr> constraints;
            RobotPtr robot;

            int maxIterations;
            int currentIteration;
            bool running;
    };

    typedef boost::shared_ptr<ConstrainedIK> ConstrainedIKPtr;
}

#endif
