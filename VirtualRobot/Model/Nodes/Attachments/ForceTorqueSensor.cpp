
#include "ForceTorqueSensor.h"

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
        return "ForceTorqueSensor";
    }

    ModelNodeAttachmentPtr ForceTorqueSensor::clone()
    {
        ModelNodeAttachmentPtr result(new ForceTorqueSensor(name, localTransformation, visualizationType));
        return result;
    }

    void ForceTorqueSensor::initVisualization()
    {
        VisualizationFactoryPtr factory;
        if (visualizationType.empty())
        {
            factory = VirtualRobot::VisualizationFactory::first(NULL);
        } else
        {
            factory = VirtualRobot::VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }
}

