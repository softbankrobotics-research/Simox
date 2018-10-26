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
#pragma once

#include "../../../Visualization/Visualization.h"
#include "ModelNodeAttachment.h"

#include <cstdint>
#include <string>

namespace VirtualRobot
{
    class FrameAttachment : public ModelNodeAttachment
    {
        friend class FrameAttachmentFactory;

    protected:
        /*!
         * Constructor.
         * \param name  The name of the attachment.
         * \param localTransform    The transformation to apply to the attachment's pose after attaching to a ModelNode.
         */
        FrameAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation = Eigen::Matrix4f::Identity());

    public:
        /*!
         * Destructor.
         */
        virtual ~FrameAttachment() override;

        /*!
         * Checks if this attachment is attachable to the given node.
         * Mostly determined on the basis of the node type.
         *
         * @param node The node to check, if this attachment is attachable.
         *
         * @return True, if this attachment is attachable; false otherwise.
         */
        virtual bool isAttachable(const ModelNodePtr &node) const override;

        /*!
         * Get the type of this attachment.
         * This is used to seperate different attached attachments.
         *
         * @return The type of this attachment.
         */
        virtual std::string getType() const override;

        virtual ModelNodeAttachmentPtr clone() const override;
    };

    using FrameAttachmentPtr = std::shared_ptr<FrameAttachment>;
}
