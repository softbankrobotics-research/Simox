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
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelFrame_h_
#define _VirtualRobot_Modelframe_h_

#include "ModelNodeAttachment.h"

namespace VirtualRobot
{
    class ModelFrame : public ModelNodeAttachment
    {
        friend class ModelNode;
        friend class ModelFrameFactory;

    protected:
        /*!
         * Constructor.
         */
        ModelFrame(const std::string &name, const Eigen::Matrix4f &localTransformation = Eigen::Matrix4f::Identity(), std::string visualizationType = "");

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelFrame();

        /*!
         * Checks if this attachment is attachable to the given node.
         * Mostly determined on the basis of the node type.
         *
         * @param node The node to check, if this attachment is attachable.
         *
         * @return True, if this attachment is attachable; false otherwise.
         */
        virtual bool isAttachable(ModelNodePtr node);

        /*!
         * Get the type of this attachment.
         * This is used to seperate different attached attachments.
         *
         * @return "ModelFrame".
         */
        virtual std::string getType();

    private:
        void initVisualization(std::string visualizationType);
    };
    
    typedef std::shared_ptr<ModelFrame> ModelFramePtr;
}

#endif
