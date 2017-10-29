
#include "Sensor.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    Sensor::Sensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
        initVisualization();
    }


    Sensor::~Sensor()
    {
    }


    bool Sensor::isAttachable(ModelNodePtr node)
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

    void Sensor::initVisualization()
    {
        VisualizationFactoryPtr factory = VisualizationFactory::getGlobalVisualizationFactory();

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }
}

