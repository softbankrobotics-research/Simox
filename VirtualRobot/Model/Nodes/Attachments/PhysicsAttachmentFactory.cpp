#include "PhysicsAttachmentFactory.h"

namespace  VirtualRobot
{
    // register this class in the super class factory
    PhysicsAttachmentFactory::SubClassRegistry PhysicsAttachmentFactory::registry(PhysicsAttachmentFactory::getName(), &PhysicsAttachmentFactory::createInstance);

    std::string VirtualRobot::PhysicsAttachmentFactory::getName()
    {
        return "PhysicsAttachment";
    }

    std::shared_ptr<ModelNodeAttachmentFactory> PhysicsAttachmentFactory::createInstance(void *)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> factory(new PhysicsAttachmentFactory());
        return factory;
    }

    ModelNodeAttachmentPtr PhysicsAttachmentFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform,
                                                             std::string visualizationType)
    {
        PhysicsAttachmentPtr attachment(new PhysicsAttachment(name, localTransform, visualizationType));
        return attachment;
    }
}



