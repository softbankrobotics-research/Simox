
#include "ModelFrame.h"

namespace VirtualRobot
{
    ModelFrame::ModelFrame(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
        initVisualization(visualizationType);
    };


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

    void ModelFrame::initVisualization(std::string visualizationType)
    {
        if (visualizationType.empty())
        {
            return;
        }

        VisualizationFactoryPtr factory = VirtualRobot::VisualizationFactory::fromName(visualizationType, NULL);
        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
            return;
        }

        std::string name = getName();
        setVisualization(factory->createCoordSystem(1, &name));
    }
}

