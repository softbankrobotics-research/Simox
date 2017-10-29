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
#ifndef _VirtualRobot_ModelNodeAttachment_h_
#define _VirtualRobot_ModelNodeAttachment_h_

#include "../../Model.h"
#include "../../Frame.h"
#include "../../../Visualization/VisualizationNode.h"

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
         * \param \param visualizationType The name of the VisualizationFactory (@see VisualizationFactory::fromName()) to use.
         */
        ModelNodeAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation = Eigen::Matrix4f::Identity(), std::string visualizationType = "");

    public:
        /*!
         * Destructor.
         */
        virtual ~ModelNodeAttachment();

        /*!
         * Checks if this attachment is attachable to the given node.
         * Mostly determined on the basis of the node type.
         *
         * @param node The node to check, if this attachment is attachable.
         *
         * @return True, if this attachment is attachable; false otherwise.
         */
        virtual bool isAttachable(const ModelNodePtr &node);

        /*!
         * Update the values of the Attachment.
         * This method is called by the node to inform the attachment about a change.
         */
        /*virtual void update()
        {
            ModelNodePtr nodeShared = node.lock();

            if (nodeShared)
            {
                updatePose(nodeShared->getGlobalPose());
            }
        }*/
        virtual void update(const Eigen::Matrix4f &parentPose)
        {
            WriteLockPtr lock = getModel()->getWriteLock();
            this->globalPose = parentPose * localTransformation;
            if (visu)
                visu->setGlobalPose(globalPose);
        }

        /*!
         * Get the globalPose of this node.
         *
         * This call locks the model's mutex.
         *
         * @return The global pose.
         */
        virtual Eigen::Matrix4f getGlobalPose() const override;

        /*!
         * Get the visualisation of this attachment.
         * If this attachment does not have a visualisation, a null pointer is returned.
         *
         * @return The visualisation.
         */
        virtual VisualizationNodePtr getVisualisation()
        {
            return visu;
        }

        virtual void setVisualization(VisualizationNodePtr visu)
        {
            this->visu = visu;
        }

        /*!
         * Get the type of this attachment.
         * This is used to seperate different attached attachments.
         *
         * @return The type of this attachment.
         */
        virtual std::string getType();

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

        virtual ModelNodeAttachmentPtr clone();

        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", int tabs = 3);

    protected:

        virtual void initVisualization();

        virtual void setParent(const ModelNodePtr &node)
        {
            if (node)
            {
                this->node = node;
            }
            else
            {
                this->node.reset();
            }
        }

        ModelNodeWeakPtr node;
        Eigen::Matrix4f localTransformation;
        std::string visualizationType;
        VisualizationNodePtr visu;
    };
}


#endif // _VirtualRobot_ModelNodeAttachment_h_
