#include "ModelStructureFactory.h"

namespace VirtualRobot
{
    ModelStructureFactory::ModelStructureFactory()
    {

    }

    ModelStructureFactory::~ModelStructureFactory()
    {

    }

    ModelNodeAttachmentPtr ModelStructureFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform) const
    {
        ModelStructurePtr m(new ModelStructure(name, localTransform));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry ModelStructureFactory::registry(ModelStructureFactory::getName(), &ModelStructureFactory::createInstance);

    std::string ModelStructureFactory::getName()
    {
        return "ModelStructure";
    }

    std::shared_ptr<ModelNodeAttachmentFactory> ModelStructureFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new ModelStructureFactory());
        return attachFactory;
    }
}
