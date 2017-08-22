
#include "ModelFrame.h"

namespace VirtualRobot
{
    ModelFrame::ModelFrame(const std::string &name, const Eigen::Matrix4f &localTransformation, VisualizationNodePtr visualization)
        : ModelNodeAttachment(name, localTransformation, visualization)
    {
    };


    ModelFrame::~ModelFrame()
    {
    }


    bool ModelFrame::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ModelFrame::getType()
    {
        return "ModelFrame";
    }
}

