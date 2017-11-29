#include "ModelStructure.h"

#include "../ModelJointRevolute.h"
#include "../ModelJointPrismatic.h"
#include "../ModelJointFixed.h"

#include "../../../Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    VirtualRobot::ModelStructure::ModelStructure(const std::string &name, const Eigen::Matrix4f &localTransformation)
        : ModelNodeAttachment(name, localTransformation)
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
        VisualizationFactoryPtr factory = VisualizationFactory::getInstance();
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

    VisualizationPtr ModelStructure::createJointVisualization(ModelJointPtr joint, VisualizationFactoryPtr factory)
    {
        return factory->createSphere(7.5f);
    }

    VisualizationPtr ModelStructure::createLinkVisualization(ModelLinkPtr link, VisualizationFactoryPtr factory)
    {
        VisualizationPtr v;
        ModelNodePtr parent = link->getParentNode(ModelNode::ModelNodeType::Joint);
        if (parent)
        {
            std::vector<ModelNodePtr> children = link->getChildNodes(ModelNode::ModelNodeType::Joint);
            std::vector<Eigen::Matrix4f> from, to;
            for (const auto & child : children)
            {
                from.push_back(link->toLocalCoordinateSystem(parent->getGlobalPose()));
                to.push_back(link->toLocalCoordinateSystem(child->getGlobalPose()));
            }
            v = factory->createLineSet(from, to, 2.f);
        }
        return v;
    }

    ModelNodeAttachmentPtr ModelStructure::clone()
    {
        ModelNodeAttachmentPtr result(new ModelStructure(name, localTransformation));
        return result;
    }

    std::string ModelStructure::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        // no xml sttring for this type of attachment
        return std::string();
    }

}































