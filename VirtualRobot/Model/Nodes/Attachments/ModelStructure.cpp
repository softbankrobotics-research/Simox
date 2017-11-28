#include "ModelStructure.h"

#include "../ModelJointRevolute.h"
#include "../ModelJointPrismatic.h"
#include "../ModelJointFixed.h"

#include "../../../Visualization/VisualizationFactory.h"

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
        VisualizationFactoryPtr factory = VisualizationFactory::getGlobalVisualizationFactory();

        if (!factory)
        {
            VR_ERROR << "Could not create VisualizationFactory with type " << visualizationType << endl;
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

    VisualizationPtr ModelStructure::createJointVisualization(ModelJointPtr joint, VisualizationFactoryPtr factory)
    {
        VisualizationPtr v;
        if (joint->getType() == ModelNode::ModelNodeType::JointRevolute)
        {
            ModelJointRevolutePtr revoluteJoint = std::static_pointer_cast<ModelJointRevolute>(joint);
            Eigen::Vector3f rotationAxis = revoluteJoint->getJointRotationAxisInJointCoordSystem();

            v = factory->createCylinder(5, 15);

            // rotate the ellipse back to its initial orientation.
            Eigen::Matrix4f rotation = revoluteJoint->getGlobalPose().inverse();
            rotation.block<3,1>(0,3) = Eigen::Vector3f::Zero();
            v->applyDisplacement(rotation);
            // here , the ellipse is now aligned parallel to the global y-axis.
            // Now rotate it such that it is aligned parallel to the joint's rotation axis.
            rotation.setIdentity();
            Eigen::Vector3f alignment;
            alignment(0) = rotationAxis(1);
            alignment(1) = rotationAxis(2);
            alignment(2) = rotationAxis(0);
            Eigen::Affine3f transform(Eigen::AngleAxisf(float(M_PI / 2.0), alignment));
            rotation = transform * rotation;
            v->applyDisplacement(rotation);
        }
        else
        {
            // wip
        }
        return v;
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
        ModelNodeAttachmentPtr result(new ModelStructure(name, localTransformation,visualizationType));
        return result;
    }

    std::string ModelStructure::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs)
    {
        // no xml sttring for this type of attachment
        return std::string();
    }

}































