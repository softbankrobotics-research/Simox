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
#ifndef _VirtualRobot_Coordinate_h_
#define _VirtualRobot_Coordinate_h_

#include <VirtualRobot/VirtualRobot.h>
#include <Eigen/Geometry>


namespace VirtualRobot
{
        class VIRTUAL_ROBOT_IMPORT_EXPORT Coordinate
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Coordinate();
            ~Coordinate();

            /*!
             * \brief getGlobalPose Returns the pose in global coordinate system.
             * \return
             */
            virtual Eigen::Matrix4f getGlobalPose() const;

            /** Conveniently returns the translational part of the actual pose stored in this object.
             */
            virtual Eigen::Vector3f getGlobalPosition() const;

        protected:
            Eigen::Matrix4f globalPose;
        };

        typedef boost::shared_ptr<Coordinate> CoordinatePtr;

} //namespace VirtualRobot

#endif
