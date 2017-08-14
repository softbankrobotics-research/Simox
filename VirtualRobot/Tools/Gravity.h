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
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _VirtualRobot_Gravity_h_
#define _VirtualRobot_Gravity_h_

#include "../VirtualRobot.h"

namespace VirtualRobot
{
    class Gravity
    {
    public:
        Gravity(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rnsJoints, VirtualRobot::RobotNodeSetPtr rnsBodies);
        virtual ~Gravity();

        /*!
         * \brief computeGravityTorque Computes the torque that is needed to counteract gravity.
         * \return name value map with the computed torques
         */
        std::map<std::string, float> computeGravityTorque();

    protected:
        VirtualRobot::RobotPtr robot;

        // this rns is used to update the current pose of the robot
        VirtualRobot::RobotNodeSetPtr rns;

        // this rns is used to update the current pose of the robot
        VirtualRobot::RobotNodeSetPtr rnsBodies;
    };

    typedef boost::shared_ptr<Gravity> GravityPtr;
}

#endif
