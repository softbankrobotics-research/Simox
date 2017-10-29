
#include "PositionSensor.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    PositionSensor::PositionSensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Sensor(name, localTransformation, visualizationType)
    {
        initVisualization();
    }


    PositionSensor::~PositionSensor()
    {
    }


    bool PositionSensor::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string PositionSensor::getType()
    {
        return "position";
    }

    ModelNodeAttachmentPtr PositionSensor::clone()
    {
        ModelNodeAttachmentPtr result(new PositionSensor(name, localTransformation, visualizationType));
        return result;
    }

    void PositionSensor::initVisualization()
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

