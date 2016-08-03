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
* @author     Adrian Knobloch
* @copyright  2016 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelJointRevolute_h_
#define _VirtualRobot_ModelJointRevolute_h_

#include "ModelJoint.h"

namespace VirtualRobot
{
    class ModelJointRevolute : ModelJoint
    {
    protected:
        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         * @param jointLimitLo The lower limit of this joint.
         * @param jointLimitHi The upper limit of this joint.
         * @param jointValueOffset The offset for the value of this joint.
         * @param axis The axis of this joint.
         */
        ModelJointRevolute(ModelWeakPtr model,
                           const std::string& name,
                           Eigen::Matrix4f& localTransformation,
                           float jointLimitLo,
                           float jointLimitHi,
                           float jointValueOffset = 0.0f,
                           const Eigen::Vector3f& axis);
    public:
        /*!
         * Destructor.
         */
        virtual ~ModelJointRevolute();

        virtual ModelNodeType getType()
        {
            return ModelNode::ModelNodeType::JointRevolute;
        }

        /*!
         * Get the joint axis in defined coordinate system.
         *
         * @param coordSystem The coordinate system to get the axis in.
         * @return The axis in the given coordinate system.
        */
        Eigen::Vector3f getJointRotationAxis(Eigen::Matrix4f coordSystem) const;

        /*!
         * This is the original joint axis, without any transformations applied.
         *
         * @return The joint axis in the local coordinate system of this node.
         */
        Eigen::Vector3f getJointRotationAxisInJointCoordSystem() const;

        /*!
         * This calculates the spatial distance between the parent of a revolute joint and a given child with the joint set to a given angle (e.g. the length of a muscle-tendon complex attached to the parent and the given child).
         *
         * @param child The child node.
         * @param angle The angle of the revolute joint in radians.
         * @return The spatial distance between parent and given child at given angle.
         */
        virtual float getLMTC(float angle);

        /*!
         * This calculates the spatial length of a moment arm defined through the triangle given by the node's parent, the specified child and the specified angle at the revolulte joint.
         *
         * @param child The child node.
         * @param angle The angle of the revolute joint in radians.
         * @return The spatial length of the moment arm.
         */
        virtual float getLMomentArm(float angle);
    };
}

#endif // _VirtualRobot_ModelJointRevolute_h_
