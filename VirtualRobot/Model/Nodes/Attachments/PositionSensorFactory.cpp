
#include "PositionSensorFactory.h"
#include "PositionSensor.h"

namespace VirtualRobot
{
    PositionSensorFactory::PositionSensorFactory()
    {
    }
        
    PositionSensorFactory::~PositionSensorFactory()
    {
    }

    ModelNodeAttachmentPtr PositionSensorFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        PositionSensorPtr m(new PositionSensor(name, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry PositionSensorFactory::registry(PositionSensorFactory::getName(), &PositionSensorFactory::createInstance);


    /**
    * \return "position"
    */
    std::string PositionSensorFactory::getName()
    {
        return "position";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> PositionSensorFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new PositionSensorFactory());
        return attachFactory;
    }

}

