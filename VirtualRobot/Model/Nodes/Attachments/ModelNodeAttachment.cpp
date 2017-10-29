
#include "ModelNodeAttachment.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    ModelNodeAttachment::ModelNodeAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : Frame(name), localTransformation(localTransformation), visualizationType(visualizationType)
    {
        initVisualization();
    }


    ModelNodeAttachment::~ModelNodeAttachment()
    {
    }


    bool ModelNodeAttachment::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ModelNodeAttachment::getType()
    {
        return "ModelNodeAttachment";
    }

    ModelNodeAttachmentPtr ModelNodeAttachment::clone()
    {
        ModelNodeAttachmentPtr result(new ModelNodeAttachment(name, localTransformation,visualizationType));
        return result;
    }

    Eigen::Matrix4f ModelNodeAttachment::getGlobalPose() const
    {
        ReadLockPtr lock = getModel()->getReadLock();
        return globalPose;
    }

    void ModelNodeAttachment::initVisualization()
    {
        VisualizationFactoryPtr factory = VisualizationFactory::getGlobalVisualizationFactory();

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }
}

