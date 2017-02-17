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
    class ModelJointRevolute : public ModelJoint
    {
    public:
        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param staticTransformation The transformation from the parent of this node to this node.
         * @param jointLimitLo The lower limit of this joint.
         * @param jointLimitHi The upper limit of this joint.
         * @param jointValueOffset The offset for the value of this joint.
         * @param axis The axis of this joint.
         */
        ModelJointRevolute(const ModelWeakPtr& model,
                           const std::string& name,
                           Eigen::Matrix4f& staticTransformation,
                           float jointLimitLo,
                           float jointLimitHi,
                           float jointValueOffset = 0.0f,
                           Eigen::Vector3f& axis);

        /*!
         * Destructor.
         */
        virtual ~ModelJointRevolute();

        virtual ModelNodeType getType() const override;

        /*!
         * Get the joint axis in defined coordinate system.
         *
         * @param coordSystem The coordinate system to get the axis in.
         * @return The axis in the given coordinate system.
        */
        Eigen::Vector3f getJointRotationAxis(const Eigen::Matrix4f& coordSystem) const;

        /*!
         * This is the original joint axis, without any transformations applied.
         *
         * @return The joint axis in the local coordinate system of this node.
         */
        Eigen::Vector3f getJointRotationAxisInJointCoordSystem() const;

        virtual Eigen::Matrix4f getNodeTransformation() const override;

    private:
        Eigen::Vector3f& axis;
    };
}

#endif // _VirtualRobot_ModelJointRevolute_h_
