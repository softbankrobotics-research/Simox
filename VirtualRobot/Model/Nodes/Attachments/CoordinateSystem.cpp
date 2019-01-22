
#include "CoordinateSystem.h"
#include "../../../Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    CoordinateSystem::CoordinateSystem(const std::string &name, const Eigen::Matrix4f &localTransformation)
            : CustomVisualizationAttachment (name, VisualizationFactory::getInstance()->createCoordSystem(), localTransformation)
    {
    }

    CoordinateSystem::~CoordinateSystem()
    {
    }

    std::string CoordinateSystem::getType() const
    {
        return "CoordinateSystem";
    }

    ModelNodeAttachmentPtr CoordinateSystem::clone() const
    {
        return ModelNodeAttachmentPtr(new CoordinateSystem(getName(), localTransformation));
    }
}

