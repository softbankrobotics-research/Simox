
#include "ModelFrameFactory.h"

namespace VirtualRobot
{
    ModelFrameFactory::ModelFrameFactory()
    {
    }
        
    ModelFrameFactory::~ModelFrameFactory()
    {
    }

    ModelNodeAttachmentPtr ModelFrameFactory::createAttachment(const std::string &name, const Eigen::Matrix4f &localTransform, std::string visualizationType)
    {
        ModelFramePtr m(new ModelFrame(name, localTransform, visualizationType));
        return m;
    }


    /**
    * register this class in the super class factory
    */
    ModelNodeAttachmentFactory::SubClassRegistry ModelFrameFactory::registry(ModelFrameFactory::getName(), &ModelFrameFactory::createInstance);


    /**
    * \return "ModelFrame"
    */
    std::string ModelFrameFactory::getName()
    {
        return "ModelFrame";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    std::shared_ptr<ModelNodeAttachmentFactory> ModelFrameFactory::createInstance(void*)
    {
        std::shared_ptr<ModelNodeAttachmentFactory> attachFactory(new ModelFrameFactory());
        return attachFactory;
    }

}

