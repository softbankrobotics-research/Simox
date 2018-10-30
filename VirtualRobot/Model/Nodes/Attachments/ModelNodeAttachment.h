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

#include "../../Model.h"
#include "../../Frame.h"
#include "../../../Visualization/Visualization.h"

#include <cstdint>
#include <string>

namespace VirtualRobot
{
    class ModelNodeAttachment : public Frame
    {
        friend class ModelNode;
        friend class ModelNodeAttachmentFactory;

    protected:
        /*!
         * Constructor.
         * \param name  The name of the attachment.
         * \param localTransform    The transformation to apply to the attachment's pose after attaching to a ModelNode.
         */
        ModelNodeAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation = Eigen::Matrix4f::Identity())
                : Frame(name), localTransformation(localTransformation)
        {}

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNodeAttachment() override = default;

        /*!
         * Checks if this attachment is attachable to the given node.
         * Mostly determined on the basis of the node type.
         *
         * @param node The node to check, if this attachment is attachable.
         *
         * @return True, if this attachment is attachable; false otherwise.
         */
        virtual bool isAttachable(const ModelNodePtr &node) const = 0;

        /*!
         * Update the values of the Attachment.
         * This method is called by the node to inform the attachment about a change.
         */
        virtual void update(const Eigen::Matrix4f &parentPose)
        {
            WriteLockPtr lock = getModel()->getWriteLock();
            this->globalPose = parentPose * localTransformation;
        }


        /*!
         * Get the visualisation of this attachment.
         * If this attachment does not have a visualisation, a null pointer is returned.
         *
         * @return The visualisation.
         */
        virtual VisualizationPtr getVisualisation() const
        {
            return nullptr;
        }

        /*!
         * Get the type of this attachment.
         * This is used to seperate different attached attachments.
         *
         * @return The type of this attachment.
         */
        virtual std::string getType() const = 0;

        /*!
         * Get the node, this attachment is attached to.
         *
         * @return The node.
         */
        ModelNodePtr getParent() const
        {
            ModelNodePtr nodeShared = node.lock();

            if (nodeShared)
            {
                return nodeShared;
            }

            return ModelNodePtr();
        }

        /*!
         * Get the model, this attachment is part of.
         *
         * @return The model.
         */
        ModelPtr getModel() const
        {
            ModelNodePtr nodeShared = node.lock();

            if (nodeShared)
            {
                return nodeShared->getModel();
            }

            return ModelPtr();
        }

        virtual ModelNodeAttachmentPtr clone() const = 0;

        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", int tabs = 3) const
        {
            return "";
        }

    protected:
        virtual void setParent(const ModelNodePtr &node)
        {
            if (node)
            {
                this->node = node;
                update(node->getGlobalPose());
            }
            else
            {
                this->node.reset();
            }
        }

        ModelNodeWeakPtr node;
        Eigen::Matrix4f localTransformation;
    };
}
