
#include "CustomVisualizationAttachmentFactory.h"

namespace VirtualRobot
{
    CustomVisualizationAttachmentFactory::CustomVisualizationAttachmentFactory()
    {
    }

    CustomVisualizationAttachmentFactory::~CustomVisualizationAttachmentFactory()
    {
    }

    ModelNodeAttachmentPtr CustomVisualizationAttachmentFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        CustomVisualizationAttachmentPtr m(new CustomVisualizationAttachment(name, nullptr, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry CustomVisualizationAttachmentFactory::registry(CustomVisualizationAttachmentFactory::getName(), &CustomVisualizationAttachmentFactory::createInstance);


    /**
    * \return "forcetorque"
    */
    std::string CustomVisualizationAttachmentFactory::getName()
    {
        return "CustomVisualizationAttachment";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> CustomVisualizationAttachmentFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new CustomVisualizationAttachmentFactory());
        return attachFactory;
    }

}

