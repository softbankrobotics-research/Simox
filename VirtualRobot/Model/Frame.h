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
#ifndef _VirtualRobot_Frame_h_
#define _VirtualRobot_Frame_h_

#include "../../VirtualRobot/VirtualRobot.h"
#include <Eigen/Geometry>


namespace VirtualRobot
{
        class VIRTUAL_ROBOT_IMPORT_EXPORT Frame
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Frame(const std::string &name = std::string());
            ~Frame();

            /*!
             * \brief getGlobalPose Returns the pose in global coordinate system.
             * \return
             */
            virtual Eigen::Matrix4f getGlobalPose() const;

            /** Conveniently returns the translational part of the actual pose stored in this object.
             */
            virtual Eigen::Vector3f getGlobalPosition() const;

            virtual std::string getName() const;
            virtual void setName(const std::string &name);

			/*!
			* Transforms the pose, given in global coordinate system, to the local coordinate system of this node.
			*
			* @param poseGlobal The pose, given in global coordinate system, that should be transformed to the local coordinate system of this node.
			* @return The transformed pose.
			*/
			Eigen::Matrix4f toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const;

			/*!
			* Transforms a position, given in global coordinate system, to the local coordinate system of this node.
			*
			* @param positionGlobal The position, given in global coordinate system, that should be transformed to the local coordinate system of this node.
			* @return The transformed position.
			*/
			Eigen::Vector3f toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const;

			/*!
			* Transforms the pose, given in local coordinate system, to the global coordinate system.
			*
			* @param poseLocal The pose, given in local coordinate system of this node, that should be transformed to the global coordinate system.
			* @return The transformed pose.
			*/
			Eigen::Matrix4f toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const;

			/*!
			* Transforms the position, given in local coordinate system, to the global coordinate system.
			*
			* @param positionLocal The position, given in local coordinate system of this node, that should be transformed to the global coordinate system.
			* @return The transformed position.
			*/
			Eigen::Vector3f toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const;

			/*!
			* Returns the transformation matrix from this object to otherObject.
			*
			* @param otherObject The object to transform to.
			* @return The transform matrix.
			*/
			Eigen::Matrix4f getTransformationTo(const FramePtr& otherObject);

			/*!
			* Returns the transformation matrix from otherObject to this object.
			*
			* @param otherObject The object to transform from.
			* @return The transform matrix.
			*/
			Eigen::Matrix4f getTransformationFrom(const FramePtr& otherObject);

        protected:
            Eigen::Matrix4f globalPose;
            std::string name;
        };

} //namespace VirtualRobot

#endif
