
#include "ModelNodeAttachment.h"
#include "../../XML/BaseIO.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

namespace VirtualRobot
{
    ModelNodeAttachment::ModelNodeAttachment(const std::string &name, const Eigen::Matrix4f &localTransformation)
        : Frame(name), localTransformation(localTransformation)
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
        ModelNodeAttachmentPtr result(new ModelNodeAttachment(name, localTransformation));
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
        VisualizationFactoryPtr factory = VisualizationFactory::getInstance();
        std::string name = getName();
        setVisualization(factory->createCoordSystem(&name));
    }

    VisualizationPtr ModelNodeAttachment::getVisualisation()
    {
        return visu;
    }

    void ModelNodeAttachment::setVisualization(VisualizationPtr visu)
    {
        this->visu = visu;
    }
}

