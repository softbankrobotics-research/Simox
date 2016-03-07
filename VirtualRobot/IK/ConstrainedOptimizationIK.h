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
* @author     Matthias Hadlich
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ConstrainedOptimizationIK_h_
#define _VirtualRobot_ConstrainedOptimizationIK_h_

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/IK/ConstrainedIK.h"

#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedOptimizationIK : public ConstrainedIK, public boost::enable_shared_from_this<ConstrainedOptimizationIK>
    {
    public:
        ConstrainedOptimizationIK(RobotPtr& robot, const RobotNodeSetPtr& nodeSet);

    protected:
        RobotNodeSetPtr nodeSet;
    };

    typedef boost::shared_ptr<ConstrainedOptimizationIK> ConstrainedOptimizationIKPtr;
}

#endif

