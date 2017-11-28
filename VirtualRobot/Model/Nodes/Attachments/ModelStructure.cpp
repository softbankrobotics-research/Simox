#include "ModelStructure.h"

#include "../ModelJointRevolute.h"
#include "../ModelJointPrismatic.h"
#include "../ModelJointFixed.h"

namespace VirtualRobot
{
    VirtualRobot::ModelStructure::ModelStructure(const std::string &name, const Eigen::Matrix4f &localTransformation, std::string visualizationType)
        : ModelNodeAttachment(name, localTransformation, visualizationType)
    {
        initVisualization();
    }

    ModelStructure::~ModelStructure()
    {

    }

    bool ModelStructure::isAttachable(const ModelNodePtr &node)
    {
        return true;
    }

    std::string ModelStructure::getType()
    {
        return "ModelStructure";
    }

    void VirtualRobot::ModelStructure::setParent(const ModelNodePtr &node)
    {
        ModelNodeAttachment::setParent(node);
        initVisualization();
    }

    void ModelStructure::initVisualization()
    {
        VisualizationFactoryPtr factory;
        if (visualizationType.empty())
        {
            factory = VirtualRobot::VisualizationFactory::first(nullptr);
        } else
        {
            factory = VirtualRobot::VisualizationFactory::fromName(visualizationType, nullptr);
        }
        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type '" << visualizationType << "'." << endl;
            return;
        }

        std::string name = getName();

        ModelNodePtr node = getParent();
        if (node)
        {
            if (node->isJoint())
            {
                setVisualization(createJointVisualization(std::static_pointer_cast<ModelJoint>(node), factory));
            }
            else if (node->isLink())
            {
                setVisualization(createLinkVisualization(std::static_pointer_cast<ModelLink>(node), factory));
            }
            else
            {
                VR_ERROR << "Could not initialize Visualization for node '" << node->getName() << "'" << endl;
            }
        }

    }

    VisualizationNodePtr ModelStructure::createJointVisualization(ModelJointPtr joint, VisualizationFactoryPtr factory)
    {
        VisualizationNodePtr v = factory->createSphere(7.5f);
        return v;
    }

    VisualizationNodePtr ModelStructure::createLinkVisualization(ModelLinkPtr link, VisualizationFactoryPtr factory)
    {
        VisualizationNodePtr v;
        ModelNodePtr parent = link->getParentNode(ModelNode::ModelNodeType::Joint);
        if (parent)
        {
            std::vector<VisualizationNodePtr> lines;
            std::vector<ModelNodePtr> children = link->getChildNodes(ModelNode::ModelNodeType::Joint);
            for (const auto & child : children)
            {
                Eigen::Matrix4f localStartPose = link->toLocalCoordinateSystem(parent->getGlobalPose());
                Eigen::Matrix4f localEndPose = link->toLocalCoordinateSystem(child->getGlobalPose());
                lines.push_back(factory->createLine(localStartPose, localEndPose, 2.0f));
            }
            v = factory->createUnitedVisualization(lines);
        }
        return v;
    }

    ModelNodeAttachmentPtr ModelStructure::clone()
    {
        ModelNodeAttachmentPtr result(new ModelStructure(name, localTransformation,visualizationType));
        return result;
    }

    std::string ModelStructure::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        // no xml sttring for this type of attachment
        return std::string();
    }

}































