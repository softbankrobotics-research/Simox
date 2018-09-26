#include "ModelStructure.h"

#include "../ModelJointRevolute.h"
#include "../ModelJointPrismatic.h"
#include "../ModelJointFixed.h"

#include "../../../Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    VirtualRobot::ModelStructure::ModelStructure(const std::string &name, const Eigen::Matrix4f &localTransformation)
        : CustomVisualizationAttachment(name, nullptr, localTransformation)
    {
        initVisualization();
    }

    ModelStructure::~ModelStructure()
    {
    }

    std::string ModelStructure::getType() const
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
        ModelNodePtr node = getParent();
        if (node)
        {
            if (node->isJoint())
            {
                setVisualization(createJointVisualization(std::static_pointer_cast<ModelJoint>(node)));
            }
            else if (node->isLink())
            {
                setVisualization(createLinkVisualization(std::static_pointer_cast<ModelLink>(node)));
            }
            else
            {
                VR_ERROR << "Could not initialize Visualization for node '" << node->getName() << "'" << endl;
            }
        }
    }

    VisualizationPtr ModelStructure::createJointVisualization(const ModelJointPtr& joint)
    {
        static auto factory = VirtualRobot::VisualizationFactory::getInstance();
        if (joint->isRotationalJoint())
        {
            auto rot = std::static_pointer_cast<ModelJointRevolute>(joint);
            auto axis = rot->getJointRotationAxis(joint);
            auto visu = factory->createCylinder(4.f, 20.f);
            Eigen::Matrix4f pose = MathTools::quat2eigen4f(MathTools::getRotation(axis, Eigen::Vector3f::UnitY()));
            visu->setGlobalPose(pose);
            return factory->createVisualisationSet({visu});
        }
        else if (joint->isTranslationalJoint())
        {
            auto pris = std::static_pointer_cast<ModelJointPrismatic>(joint);
            auto dir = pris->getJointTranslationDirection(joint);
            auto visu = factory->createBox(7.5, 7.5, 20.f);
            Eigen::Matrix4f pose = MathTools::quat2eigen4f(MathTools::getRotation(dir, Eigen::Vector3f::UnitZ()));
            visu->setGlobalPose(pose);
            return factory->createVisualisationSet({visu});
        }
        else
        {
            return factory->createSphere(7.5f);
        }
    }

    VisualizationPtr ModelStructure::createLinkVisualization(const ModelLinkPtr &link)
    {
        VisualizationPtr v;
        ModelNodePtr parent = link->getParentNode(VirtualRobot::ModelNode::Joint);
        if (!parent)
        {
            parent = link->getModel()->getRootNode();
        }
        std::vector<ModelNodePtr> children = link->getChildNodes();
        std::vector<Eigen::Matrix4f> from, to;
        for (const auto & child : children)
        {
            if (child->isJoint() || child->getChildNodes().size() == 0)
            {
                from.push_back(link->toLocalCoordinateSystem(parent->getGlobalPose()));
                to.push_back(link->toLocalCoordinateSystem(child->getGlobalPose()));
            }
        }
        return VisualizationFactory::getInstance()->createLineSet(from, to, 2.f);
    }

    ModelNodeAttachmentPtr ModelStructure::clone() const
    {
        ModelNodeAttachmentPtr result(new ModelStructure(name, localTransformation));
        return result;
    }

    std::string ModelStructure::toXML(const std::string &basePath, const std::string &modelPathRelative, int tabs) const
    {
        // no xml sttring for this type of attachment
        return std::string();
    }

}































