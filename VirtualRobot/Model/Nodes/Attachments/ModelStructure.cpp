#include "ModelStructure.h"

namespace VirtualRobot
{
    VirtualRobot::ModelStructure::ModelStructure(const std::string &name, const Eigen::Matrix4f &localTransformation, VirtualRobot::VisualizationNodePtr visualization)
        : ModelNodeAttachment(name, localTransformation, visualization)
    {

    }

    ModelStructure::~ModelStructure()
    {

    }

    bool ModelStructure::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ModelStructure::getType()
    {
        return "ModelStructure";
    }
}
