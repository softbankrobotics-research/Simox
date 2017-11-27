
#include "ModelNodeAttachmentFactory.h"

namespace VirtualRobot
{
    ModelNodeAttachmentFactory::ModelNodeAttachmentFactory()
    {
    }
        
    ModelNodeAttachmentFactory::~ModelNodeAttachmentFactory()
    {
    }

    ModelNodeAttachmentPtr ModelNodeAttachmentFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform, std::string visualizationType)
    {
        ModelNodeAttachmentPtr m(new ModelNodeAttachment(name, localTransform, visualizationType));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry ModelNodeAttachmentFactory::registry(ModelNodeAttachmentFactory::getName(), &ModelNodeAttachmentFactory::createInstance);


    /**
    * \return "modelnodeattachment"
    */
    std::string ModelNodeAttachmentFactory::getName()
    {
        return "modelnodeattachment";
    }


    std::shared_ptr<ModelNodeAttachmentFactory> ModelNodeAttachmentFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new ModelNodeAttachmentFactory());
        return attachFactory;
    }

}

