
#include "ModelFrame.h"

namespace VirtualRobot
{
    ModelFrame::ModelFrame(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
        initVisualization();
    }


    ModelFrame::~ModelFrame()
    {
    }


    bool ModelFrame::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ModelFrame::getType()
    {
        return "ModelFrame";
    }

    ModelNodeAttachmentPtr ModelFrame::clone()
    {
        ModelNodeAttachmentPtr result(new ModelFrame(name, localTransformation,visualizationType));
        return result;
    }

    void ModelFrame::initVisualization()
    {
        VisualizationFactoryPtr factory;
        if (visualizationType.empty())
        {
            factory = VirtualRobot::VisualizationFactory::first(NULL);
        } else
        {
            factory = VirtualRobot::VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }
}

