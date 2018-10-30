
#include "Sensor.h"

namespace VirtualRobot
{
    Sensor::Sensor(const std::string &name, const Eigen::Matrix4f &localTransformation)
        : ModelNodeAttachment(name, localTransformation)
    {
    }


    Sensor::~Sensor()
    {
    }
}

