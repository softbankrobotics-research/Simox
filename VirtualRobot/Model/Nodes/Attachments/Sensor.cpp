
#include "Sensor.h"

namespace VirtualRobot
{
    Sensor::Sensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
    }


    Sensor::~Sensor()
    {
    }


    bool Sensor::isAttachable(const ModelNodePtr &node)
    {
        return true;
    }

    std::string Sensor::getType()
    {
        return "Sensor";
    }

    ModelNodeAttachmentPtr Sensor::clone()
    {
        ModelNodeAttachmentPtr result(new Sensor(name, localTransformation, visualizationType));
        return result;
    }
}

