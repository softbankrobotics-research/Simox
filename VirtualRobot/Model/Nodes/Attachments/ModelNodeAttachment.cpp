
#include "ModelNodeAttachment.h"
#include "../../XML/BaseIO.h"

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


    bool ModelNodeAttachment::isAttachable(const ModelNodePtr &node)
    {
        return true;
    }

    std::string ModelNodeAttachment::getType()
    {
        return "modelnodeattachment";
    }

    ModelNodeAttachmentPtr ModelNodeAttachment::clone()
    {
        ModelNodeAttachmentPtr result(new ModelNodeAttachment(name, localTransformation,visualizationType));
        return result;
    }

    std::string ModelNodeAttachment::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }
        std::stringstream ss;
        ss << t << "<Frame name='" << name <<"'>\n";
        //ss << t << "<ModelNodeAttachment name='" << name <<"'>\n";
        std::string pre2 = t + "\t";
        std::string pre3 = pre2 + "\t";
        ss << pre2 << "<Transform>" << endl;
        ss << BaseIO::toXML(localTransformation, pre3);
        ss << pre2 << "</Transform>" << endl;
        //ss << t << "</ModelNodeAttachment>\n";
        ss << t << "</Frame>\n";

        return ss.str();
    }

    Eigen::Matrix4f ModelNodeAttachment::getGlobalPose() const
    {
        ReadLockPtr lock = getModel()->getReadLock();
        return globalPose;
    }

    void ModelNodeAttachment::initVisualization()
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

    VisualizationNodePtr ModelNodeAttachment::getVisualisation()
    {
        return visu;
    }

    void ModelNodeAttachment::setVisualization(VisualizationNodePtr visu)
    {
        this->visu = visu;
    }
}

