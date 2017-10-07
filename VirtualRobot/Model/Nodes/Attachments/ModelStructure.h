#ifndef MODELSTRUCTUREATTACHMENT_H
#define MODELSTRUCTUREATTACHMENT_H

#include "ModelNodeAttachment.h"

namespace VirtualRobot
{
    class ModelStructure : public ModelNodeAttachment
    {
        friend class ModelNode;
        friend class ModelStructureFactory;

    protected:
        ModelStructure(const std::string &name, const Eigen::Matrix4f &localTransformation = Eigen::Matrix4f::Identity(), VisualizationNodePtr visualization = VisualizationNodePtr());

    public:
        ~ModelStructure();

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
         * @return "ModelStructure".
         */
        virtual std::string getType();
    };

    typedef std::shared_ptr<ModelStructure> ModelStructurePtr;
}

#endif // MODELSTRUCTUREATTACHMENT_H
