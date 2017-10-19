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

    bool ModelStructure::isAttachable(ModelNodePtr node)
    {
        return true;
    }

    std::string ModelStructure::getType()
    {
        return "ModelStructure";
    }

    void VirtualRobot::ModelStructure::setParent(ModelNodePtr node)
    {
        ModelNodeAttachment::setParent(node);
        initVisualization();
    }

    void ModelStructure::initVisualization()
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
        }

    }

    VisualizationNodePtr ModelStructure::createJointVisualization(ModelJointPtr joint, VisualizationFactoryPtr factory)
    {
        VisualizationNodePtr v;
        if (joint->getType() == ModelNode::ModelNodeType::JointRevolute)
        {
            ModelJointRevolutePtr revoluteJoint = std::static_pointer_cast<ModelJointRevolute>(joint);
            Eigen::Vector3f rotationAxis = revoluteJoint->getJointRotationAxisInJointCoordSystem();

            v = factory->createCylinder(5, 15);

            // rotate the ellipse back to its initial orientation.
            Eigen::Matrix4f rotation = revoluteJoint->getGlobalPose().inverse();
            rotation.block<3,1>(0,3) = Eigen::Vector3f::Zero();
            factory->applyDisplacement(v, rotation);
            // here , the ellipse is now aligned parallel to the global y-axis.
            // Now rotate it such that it is aligned parallel to the joint's rotation axis.
            rotation.setIdentity();
            Eigen::Vector3f alignment;
            alignment(0) = rotationAxis(1);
            alignment(1) = rotationAxis(2);
            alignment(2) = rotationAxis(0);
            Eigen::Affine3f transform(Eigen::AngleAxisf(float(M_PI / 2.0f), alignment));
            rotation = transform * rotation;
            factory->applyDisplacement(v, rotation);
        }
        else
        {
            // wip
        }
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

}































