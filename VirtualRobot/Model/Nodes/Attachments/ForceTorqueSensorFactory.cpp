
#include "ForceTorqueSensorFactory.h"
#include "ForceTorqueSensor.h"

namespace VirtualRobot
{
    ForceTorqueSensorFactory::ForceTorqueSensorFactory()
    {
    }
        
    ForceTorqueSensorFactory::~ForceTorqueSensorFactory()
    {
    }

    ModelNodeAttachmentPtr ForceTorqueSensorFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        ForceTorqueSensorPtr m(new ForceTorqueSensor(name, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry ForceTorqueSensorFactory::registry(ForceTorqueSensorFactory::getName(), &ForceTorqueSensorFactory::createInstance);


    /**
    * \return "forcetorque"
    */
    std::string ForceTorqueSensorFactory::getName()
    {
        return "forcetorque";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> ForceTorqueSensorFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new ForceTorqueSensorFactory());
        return attachFactory;
    }

}

