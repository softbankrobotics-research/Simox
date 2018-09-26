
#include "CustomVisualizationAttachment.h"

namespace VirtualRobot
{
    CustomVisualizationAttachment::CustomVisualizationAttachment(const std::string &name, VisualizationPtr visu, const Eigen::Matrix4f &localTransformation)
            : ModelNodeAttachment (name, localTransformation), visu(visu)
    {
    }

    CustomVisualizationAttachment::~CustomVisualizationAttachment()
    {
    }

    bool CustomVisualizationAttachment::isAttachable(const ModelNodePtr &) const
    {
        return true;
    }

    void CustomVisualizationAttachment::update(const Eigen::Matrix4f &parentPose)
    {
        WriteLockPtr lock = getModel()->getWriteLock();
        this->globalPose = parentPose * localTransformation;
        if (visu)
        {
            visu->setGlobalPose(globalPose);
        }
    }

    VisualizationPtr CustomVisualizationAttachment::getVisualisation() const
    {
        return visu;
    }

    void CustomVisualizationAttachment::setVisualization(VisualizationPtr visu)
    {
        auto l = getModel()->getWriteLock();
        if (this->visu)
        {
            visu->setGlobalPose(this->visu->getGlobalPose());
        }
        this->visu = visu;
    }

    std::string CustomVisualizationAttachment::getType() const
    {
        return "CustomVisualizationAttachment";
    }

    ModelNodeAttachmentPtr CustomVisualizationAttachment::clone() const
    {
        CustomVisualizationAttachmentPtr c(new CustomVisualizationAttachment(name, visu->clone(), localTransformation));
        return c;
    }
}

