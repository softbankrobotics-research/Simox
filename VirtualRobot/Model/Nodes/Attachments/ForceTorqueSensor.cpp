
#include "ForceTorqueSensor.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    ForceTorqueSensor::ForceTorqueSensor(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Sensor(name, localTransformation, visualizationType)
    {
        initVisualization();
    }


    ForceTorqueSensor::~ForceTorqueSensor()
    {
    }


    bool ForceTorqueSensor::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ForceTorqueSensor::getType()
    {
        return "forcetorque";
    }

    ModelNodeAttachmentPtr ForceTorqueSensor::clone()
    {
        ModelNodeAttachmentPtr result(new ForceTorqueSensor(name, localTransformation, visualizationType));
        return result;
    }

    void ForceTorqueSensor::initVisualization()
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

