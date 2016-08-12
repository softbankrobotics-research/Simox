#include "ModelNode.h"
#include "../VirtualRobotException.h"
#include "../ConditionedLock.h"

#include <algorithm>

namespace VirtualRobot
{
    ModelNode::ModelNode(ModelWeakPtr model, const std::string& name, Eigen::Matrix4f& localTransformation)
        : initialized(false),
          model(model),
          name(name),
          parent(ModelNodeWeakPtr()),
          children(),
          localTransformation(localTransformation),
          globalPose(Eigen::Matrix4f::Identity()),
          attachments(),
          attachmentsWithVisualisation()

    {
    }

    ModelNode::~ModelNode()
    {
    }

    void ModelNode::initialize(ModelNodePtr parent, const std::vector<ModelNodePtr>& children)
    {
        ModelPtr modelShared = getModel();
        THROW_VR_EXCEPTION_IF(!modelShared->hasModelNode(shared_from_this()),
                              "ModelNode \"" + getName() + "\" is not registered to model \""
                              + modelShared->getName() + "\".");

        if (parent)
        {
            parent->attachChild(shared_from_this());
            // parent of this is set in attachChild
        }

        for (ModelNodePtr child : children)
        {
            attachChild(child);
        }

        updatePose(true);

        initialized = true;
    }

    ModelPtr ModelNode::getModel() const
    {
        ModelPtr modelShared = model.lock();
        THROW_VR_EXCEPTION_IF(!modelShared, "ModelNode \"" + getName() + "\" does not have a model.");
        return modelShared;
    }

    std::string ModelNode::getName() const
    {
        return name;
    }

    bool ModelNode::isInitialized() const
    {
        return initialized;
    }

    ModelNodePtr ModelNode::getChildByName(std::string& name) const
    {
        // initialisation is checked in getChildNodes
        for (ModelNodePtr child : getChildNodes())
        {
            if (child->getName() == name)
            {
                return child;
            }
        }

        return ModelNodePtr();
    }

    ModelNodePtr ModelNode::getParentNode() const
    {
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        ModelNodePtr parentShared = parent.lock();

        if (!parentShared)
        {
            return ModelNodePtr();
        }

        return parentShared;
    }

    ModelNodePtr ModelNode::getParentLink() const
    {
        // initialisation is checked in getParentNode
        ModelNodePtr parentLink = getParentNode();

        if (!parentLink)
        {
            return ModelNodePtr();
        }

        while (parentLink->getType() & 0x0F != ModelNodeType::Link)
        {
            parentLink = parentLink->getParentNode();

            if (!parentLink)
            {
                return ModelNodePtr();
            }
        }

        return parentLink;
    }

    ModelNodePtr ModelNode::getParentJoint() const
    {
        // initialisation is checked in getParentNode
        ModelNodePtr parentJoint = getParentNode();

        if (!parentJoint)
        {
            return ModelNodePtr();
        }

        while (parentJoint->getType() & 0x0F != ModelNodeType::Joint)
        {
            parentJoint = parentJoint->getParentNode();

            if (!parentJoint)
            {
                return ModelNodePtr();
            }
        }

        return parentJoint;
    }

    std::vector<ModelNodePtr> ModelNode::getChildNodes() const
    {
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        return children;
    }

    std::vector<ModelNodePtr> ModelNode::getChildJoints() const
    {
        // initialisation is checked in getChildNodes
        std::vector<ModelNodePtr> childJoints;

        for (ModelNodePtr child : getChildNodes())
        {
            if (child->getType() & 0x0F != ModelNodeType::Joint)
            {
                std::vector<ModelNodePtr> childJointsOfChild = child->getChildJoints();
                childJoints.insert(childJoints.end(), childJointsOfChild.begin(), childJointsOfChild.end());
            }
            else
            {
                childJoints.push_back(child);
            }
        }

        return childJoints;
    }

    std::vector<ModelNodePtr> ModelNode::getChildLinks() const
    {
        // initialisation is checked in getChildNodes
        std::vector<ModelNodePtr> childLinks;

        for (ModelNodePtr child : getChildNodes())
        {
            if (child->getType() & 0x0F != ModelNodeType::Link)
            {
                std::vector<ModelNodePtr> childLinksOfChild = child->getChildLinks();
                childLinks.insert(childLinks.end(), childLinksOfChild.begin(), childLinksOfChild.end());
            }
            else
            {
                childLinks.push_back(child);
            }
        }

        return childLinks;
    }

    void ModelNode::collectAllNodes(std::vector<ModelNodePtr>& storeNodes, bool clearVector) const
    {
        // initialisation is checked in getChildNodes
        if (clearVector)
        {
            storeNodes.clear();
        }

        storeNodes.push_back(static_pointer_cast<ModelNode>(shared_from_this()));

        for (ModelNodePtr child : getChildNodes())
        {
            child->collectAllNodes(storeNodes, false);
        }
    }

    std::vector<ModelNodePtr> ModelNode::getAllParents(ModelNodeSetPtr set)
    {
        // initialisation is checked in getParentNode
        std::vector<ModelNodePtr> result;

        ModelNodePtr p = shared_from_this();

        while (p = p->getParentNode())
        {
            if (set && set->hasModelNode(p))
            {
                result.push_back(p);
            }
        }

        return result;
    }

    bool ModelNode::attachChild(ModelNodePtr newNode)
    {
        VR_ASSERT(newNode);

        if (this == newNode.get())
        {
            VR_WARNING << "Trying to attach object to it self object! name: " << getName() << std::endl;
            return false;
        }

        if (hasChild(newNode))
        {
            VR_WARNING << " Trying to attach already attached object: " << getName() << "->"
                       << newNode->getName() << std::endl;
            return true; // no error
        }

        if (newNode->getParentNode())
        {
            VR_WARNING << " Trying to attach object that has already a parent: " << getName() << "->"
                       << newNode->getName() << ", child's parent:" << newNode->getParentNode()->getName() << std::endl;
            return false;
        }

        if (hasChild(newNode->getName()))
        {
            VR_ERROR << " Trying to attach object with name: " << newNode->getName() << " to "
                     << getName() << ", but a child with same name is already present!" << std::endl;
            return false;
        }

        ModelPtr modelShared = getModel();

        children.push_back(newNode);
        newNode->parent = shared_from_this();

        return true;
    }

    bool ModelNode::hasChild(ModelNodePtr node) const
    {
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        return std::find(children.begin(), children.end(), node) != children.end();
    }

    bool ModelNode::hasChild(std::string nodeName) const
    {
        // initialisation is checked in getChildNodes
        for (ModelNodePtr child : getChildNodes())
        {
            if (child->getName() == nodeName)
            {
                return true;
            }
        }

        return false;
    }

    bool ModelNode::detachChild(ModelNodePtr node)
    {
        if (hasChild(node))
        {
            children.erase(std::find(children.begin(), children.end(), node));
            node->parent.reset();
            return true;
        }

        return false;
    }

    bool ModelNode::detachChild(std::string& nodeName)
    {
        if (hasChild(nodeName))
        {
            ModelNodePtr node = getChildByName(nodeName);
            children.erase(std::find(children.begin(), children.end(), node));
            node->parent.reset();
            return true;
        }

        return false;
    }

    Eigen::Matrix4f ModelNode::getLocalTransformation() const
    {
        return localTransformation;
    }

    void ModelNode::setLocalTransformation(const Eigen::Matrix4f& localTransformation, bool updatePose)
    {
        this->localTransformation = localTransformation;

        if (updatePose)
        {
            this->updatePose(true);
        }
    }

    Eigen::Matrix4f ModelNode::getGlobalPose() const
    {
        ReadLockPtr lock = getModel()->getReadLock();
        return globalPose;
    }

    Eigen::Matrix4f ModelNode::getPoseInRootFrame() const
    {
        ModelPtr r = getModel();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystem(globalPose);
    }

    Eigen::Vector3f ModelNode::getPositionInRootFrame() const
    {
        ModelPtr r = getModel();
        ReadLockPtr lock = r->getReadLock();
        return r->getRootNode()->toLocalCoordinateSystemVec(globalPose.block(0, 3, 3, 1));
    }

    void ModelNode::updatePose(bool updateChildren, bool updateAttachments)
    {
        ModelNodePtr parentShared = getParentNode();

        if (parentShared)
        {
            this->globalPose = parentShared->getGlobalPose() * localTransformation;
        }
        else // this is the root node
        {
            ModelPtr modelShared = getModel();
            VR_ASSERT(modelShared->getRootNode().get() == this);
            this->globalPose = modelShared->getGlobalPose() * localTransformation;
        }

        updatePoseInternally(updateChildren, updateAttachments);

        if (updateAttachments)
        {
            for (auto it = attachments.begin(); it != attachments.end(); it++)
            {
                for (ModelNodeAttachmentPtr attachment : it->second)
                {
                    attachment->update();
                }
            }
        }

        if (updateChildren)
        {
            for (ModelNodePtr child : getChildNodes())
            {
                child->updatePose(true, updateAttachments);
            }
        }
    }

    void updatePoseInternally(bool updateChildren, bool updateAttachments)
    {
    }

    bool ModelNode::attach(ModelNodeAttachmentPtr attachment)
    {
        if (!attachment || isAttached(attachment) || !attachment->isAttachable(shared_from_this()))
        {
            return false;
        }

        // first setup the attachment
        attachment->setNode(shared_from_this());
        attachment->update();

        attachments[attachment->getType()].push_back(attachment);

        if (attachment->getVisualisation())
        {
            attachmentsWithVisualisation.push_back(attachment);
        }

        return true;
    }

    bool ModelNode::isAttached(ModelNodeAttachmentPtr attachment)
    {
        if (!attachment)
        {
            return false;
        }

        std::vector<ModelNodeAttachmentPtr> allWithType = attachments[attachment->getType()];
        return std::find(allWithType.begin(), allWithType.end(), attachment) != allWithType.end();
    }

    bool ModelNode::detach(ModelNodeAttachmentPtr attachment)
    {
        if (isAttached(attachment))
        {
            attachment->setNode(ModelNodePtr());

            std::vector<ModelNodeAttachmentPtr> allWithType = attachments[attachment->getType()];
            allWithType.erase(std::find(allWithType.begin(), allWithType.end(), attachment));

            if (attachment->getVisualisation())
            {
                attachmentsWithVisualisation.erase(std::find(attachmentsWithVisualisation.begin(),
                                                   attachmentsWithVisualisation.end(), attachment));
            }

            return true;
        }

        return false;
    }

    std::vector<ModelNodeAttachmentPtr> ModelNode::getAttachments(std::string type)
    {
        if (type == "")
        {
            std::vector<ModelNodeAttachmentPtr> all;
            for (auto it = attachments.begin(); it != attachments.end(); it++)
            {
                all.reserve(all.size() + it->second.size()); // allocate memory
                all.insert(all.end(), it->second.begin(), it->second.end());
            }
            return all;
        }
        else
        {
            return attachments[type];
        }
    }

    std::vector<ModelNodeAttachmentPtr> ModelNode::getAttachmentsWithVisualisation() const
    {
        return attachmentsWithVisualisation;
    }

    void ModelNode::print(bool printChildren, bool printDecoration) const
    {
        // TODO
    }

    std::string ModelNode::toXML(const std::string& basePath, const std::string& modelPathRelative, bool storeAttachments)
    {
        return std::string(); // TODO
    }

    ModelNodePtr ModelNode::clone(ModelPtr newModel, bool cloneChildren, RobotNodePtr initializeWithParent, CollisionCheckerPtr colChecker, float scaling)
    {
        return ModelNodePtr(); // TODO
    }

    Eigen::Matrix4f ModelNode::toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const
    {
        return getGlobalPose().inverse() * poseGlobal;
    }

    Eigen::Vector3f ModelNode::toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionGlobal;
        t = toLocalCoordinateSystem(t);
        Eigen::Vector3f result = t.block(0, 3, 3, 1);
        return result;
    }

    Eigen::Matrix4f ModelNode::toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const
    {
        return getGlobalPose() * poseLocal;
    }

    Eigen::Vector3f ModelNode::toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionLocal;
        t = toGlobalCoordinateSystem(t);
        Eigen::Vector3f result = t.block(0, 3, 3, 1);
        return result;
    }

    Eigen::Matrix4f ModelNode::getTransformationTo(const ModelNodePtr otherObject)
    {
        return getGlobalPose().inverse() * otherObject->getGlobalPose();
    }

    Eigen::Matrix4f ModelNode::getTransformationFrom(const ModelNodePtr otherObject)
    {
        return otherObject->getGlobalPose().inverse() * getGlobalPose();
    }
}