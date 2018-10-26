
#include "FrameAttachment.h"

namespace VirtualRobot
{
    FrameAttachment::FrameAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation)
            : ModelNodeAttachment (name, localTransformation)
    {
    }

    FrameAttachment::~FrameAttachment()
    {
    }

    bool FrameAttachment::isAttachable(const ModelNodePtr &) const
    {
        return true;
    }

    std::string FrameAttachment::getType() const
    {
        return "FrameAttachment";
    }

    ModelNodeAttachmentPtr FrameAttachment::clone() const
    {
        FrameAttachmentPtr c(new FrameAttachment(name, localTransformation));
        return c;
    }
}

