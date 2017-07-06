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
#ifndef _VirtualRobot_Orientation_h_
#define _VirtualRobot_Orientation_h_

#include "../VirtualRobot.h"
#include "MathTools.h"

#include <Eigen/Core>
#include <vector>

namespace VirtualRobot
{

    /*!
        An axis oriented bounding box.
        Todo: Some parts of this class are similar to MathTools::OOBB.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Orientation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Orientation();
        ~Orientation();

        //! Construct from matrix
        Orientation(const Eigen::Matrix3f &rotationMatrix);

        //! Construct from quaternion
        Orientation(const MathTools::Quaternion &q);

        //! Construct from roll pitch yaw
        Orientation(const Eigen::Vector3f &rpy);

        Eigen::Vector3f rpy() const;
        Eigen::Matrix3f matrix() const;
        MathTools::Quaternion quaternion() const;

    protected:
        Eigen::Matrix3f rot;
    };

} // namespace VirtualRobot

#endif
