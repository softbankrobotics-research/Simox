
#include "FrameAttachmentFactory.h"

namespace VirtualRobot
{
    FrameAttachmentFactory::FrameAttachmentFactory()
    {
    }

    FrameAttachmentFactory::~FrameAttachmentFactory()
    {
    }

    ModelNodeAttachmentPtr FrameAttachmentFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        FrameAttachmentPtr m(new FrameAttachment(name, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry FrameAttachmentFactory::registry(FrameAttachmentFactory::getName(), &FrameAttachmentFactory::createInstance);


    /**
    * \return "forcetorque"
    */
    std::string FrameAttachmentFactory::getName()
    {
        return "FrameAttachment";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> FrameAttachmentFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new FrameAttachmentFactory());
        return attachFactory;
    }

}

