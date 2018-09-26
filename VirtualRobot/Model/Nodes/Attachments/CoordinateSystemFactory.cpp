
#include "CoordinateSystemFactory.h"

namespace VirtualRobot
{
    CoordinateSystemFactory::CoordinateSystemFactory()
    {
    }

    CoordinateSystemFactory::~CoordinateSystemFactory()
    {
    }

    ModelNodeAttachmentPtr CoordinateSystemFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        CoordinateSystemPtr m(new CoordinateSystem(name, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry CoordinateSystemFactory::registry(CoordinateSystemFactory::getName(), &CoordinateSystemFactory::createInstance);


    /**
    * \return "forcetorque"
    */
    std::string CoordinateSystemFactory::getName()
    {
        return "CoordinateSystem";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> CoordinateSystemFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new CoordinateSystemFactory());
        return attachFactory;
    }

}

