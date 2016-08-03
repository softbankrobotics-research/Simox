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
#ifndef _VirtualRobot_ModelJointFixed_h_
#define _VirtualRobot_ModelJointFixed_h_

#include "ModelJoint.h"

namespace VirtualRobot
{
    class ModelJointFixed : public ModelJoint
    {
    protected:
        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         */
        ModelJointFixed(ModelWeakPtr model,
                        const std::string& name,
                        Eigen::Matrix4f& localTransformation);

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelJointFixed();

        virtual ModelNodeType getType()
        {
            return ModelNode::ModelNodeType::JointFixed;
        }
    };
}


#endif // _VirtualRobot_ModelJointFixed_h_
