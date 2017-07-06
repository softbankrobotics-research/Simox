#include "ModelNode.h"
#include "../Model.h"
#include "../VirtualRobotException.h"
#include "../Tools/ConditionedLock.h"

namespace VirtualRobot
{
    ModelNode::ModelNode(const ModelWeakPtr& model, const std::string& name, Eigen::Matrix4f& staticTransformation)
        : initialized(false),
          model(model),
          name(name),
          parent(ModelNodeWeakPtr()),
          children(),
          staticTransformation(staticTransformation),
          globalPose(Eigen::Matrix4f::Identity()),
          attachments(),
          attachmentsWithVisualisation()

    {
    }

    ModelNode::~ModelNode()
    {
    }

    void ModelNode::initialize(const ModelNodePtr& parent, const std::vector<ModelNodePtr>& children)
    {
        THROW_VR_EXCEPTION_IF(isInitialized(), "ModelNode " + getName() + "is already initialized.");
        ModelPtr modelShared = getModel();
        THROW_VR_EXCEPTION_IF(!modelShared->hasModelNode(shared_from_this()),
                              "ModelNode \"" + getName() + "\" is not registered to model \""
                              + modelShared->getName() + "\".");

        if (parent)
        {
            parent->attachChild(shared_from_this());
            // parent of this is set in attachChild
        }

        for (const ModelNodePtr & child : children)
        {
            attachChild(child);
        }

        updatePose(true);

        // i think no lock is needed for "initialized"
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
        // never updated -> no lock needed
        return name;
    }

    bool ModelNode::isInitialized() const
    {
        // only updated in "initialize" -> no lock needed
        return initialized;
    }

    ModelNodePtr ModelNode::getChildByName(const std::string& name) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        // initialisation is checked in getChildNodes
        for (const ModelNodePtr & child : getChildNodes())
        {
            if (child->getName() == name)
            {
                return child;
            }
        }

        return ModelNodePtr();
    }

    ModelNodePtr ModelNode::getParentNode(ModelNodeType type) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        ModelNodePtr parentLink = parent.lock();

        if (!parentLink)
        {
            return ModelNodePtr();
        }

        while (!checkNodeOfType(parentLink, type))
        {
            parentLink = parentLink->parent.lock();

            if (!parentLink)
            {
                return ModelNodePtr();
            }
        }

        return parentLink;
    }

    std::vector<ModelNodePtr> ModelNode::getChildNodes(ModelNodeType type) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        if (type == ModelNode::ModelNodeType::Node)
        {
            return children;
        }

        std::vector<ModelNodePtr> childLinks;

        for (const ModelNodePtr & child : children)
        {
            if (!checkNodeOfType(child, type))
            {
                std::vector<ModelNodePtr> childLinksOfChild = child->getChildNodes(type);
                childLinks.insert(childLinks.end(), childLinksOfChild.begin(), childLinksOfChild.end());
            }
            else
            {
                childLinks.push_back(child);
            }
        }

        return childLinks;
    }

    void ModelNode::collectAllNodes(std::vector<ModelNodePtr>& storeNodes, ModelNodeType type, bool clearVector)
    {
        ReadLockPtr r = getModel()->getReadLock();
        // initialisation is checked in getChildNodes
        if (clearVector)
        {
            storeNodes.clear();
        }

        if (checkNodeOfType(shared_from_this(), type))
        {
            storeNodes.push_back(shared_from_this());
        }

        for (const ModelNodePtr & child : getChildNodes())
        {
            child->collectAllNodes(storeNodes, type, false);
        }
    }

    std::vector<ModelNodePtr> ModelNode::getAllParents(const ModelNodeSetPtr& set, ModelNodeType type)
    {
        ReadLockPtr r = getModel()->getReadLock();
        // initialisation is checked in getParentNode
        std::vector<ModelNodePtr> result;

        ModelNodePtr p = shared_from_this();

        while (p = p->getParentNode(type))
        {
            if (!set || set->hasModelNode(p))
            {
                result.push_back(p);
            }
        }

        return result;
    }

    bool ModelNode::attachChild(const ModelNodePtr& newNode)
    {
        VR_ASSERT(newNode);

        if (this == newNode.get())
        {
            VR_WARNING << "Trying to attach object to it self object! name: " << getName() << std::endl;
            return false;
        }

        WriteLockPtr w = getModel()->getWriteLock();
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

        WriteLockPtr w = getModel()->getWriteLock();

        children.push_back(newNode);
        newNode->parent = shared_from_this();

        return true;
    }

    bool ModelNode::hasChild(const ModelNodePtr& node) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

        return std::find(children.begin(), children.end(), node) != children.end();
    }

    bool ModelNode::hasChild(const std::string& nodeName) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        // initialisation is checked in getChildNodes
        for (const ModelNodePtr & child : getChildNodes())
        {
            if (child->getName() == nodeName)
            {
                return true;
            }
        }

        return false;
    }

    bool ModelNode::detachChild(const ModelNodePtr& node)
    {
        if (hasChild(node))
        {
            WriteLockPtr w = getModel()->getWriteLock();
            children.erase(std::find(children.begin(), children.end(), node));
            node->parent.reset();
            return true;
        }

        return false;
    }

    bool ModelNode::detachChild(const std::string& nodeName)
    {
        ModelNodePtr node = getChildByName(nodeName);
        if (node)
        {
            WriteLockPtr w = getModel()->getWriteLock();
            children.erase(std::find(children.begin(), children.end(), node));
            node->parent.reset();
            return true;
        }
        return false;
    }

    Eigen::Matrix4f ModelNode::getStaticTransformation() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return staticTransformation;
    }

    Eigen::Matrix4f ModelNode::getNodeTransformation() const
    {
        return getStaticTransformation();
    }

    void ModelNode::setStaticTransformation(const Eigen::Matrix4f& staticTransformation, bool updatePose)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        this->staticTransformation = staticTransformation;

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
        WriteLockPtr w = getModel()->getWriteLock();
        ModelNodePtr parentShared = getParentNode();

        if (parentShared)
        {
            this->globalPose = parentShared->getGlobalPose() * getNodeTransformation();
        }
        else // this is the root node
        {
            ModelPtr modelShared = getModel();
            VR_ASSERT(modelShared->getRootNode().get() == this);
            this->globalPose = modelShared->getGlobalPose() * getNodeTransformation();
        }

        updatePoseInternally(updateChildren, updateAttachments);

        if (updateAttachments)
        {
            for (auto it = attachments.begin(); it != attachments.end(); it++)
            {
                for (const ModelNodeAttachmentPtr & attachment : it->second)
                {
                    attachment->update();
                }
            }
        }

        if (updateChildren)
        {
            for (const ModelNodePtr & child : getChildNodes())
            {
                child->updatePose(true, updateAttachments);
            }
        }
    }

    void updatePoseInternally(bool updateChildren, bool updateAttachments)
    {
    }

    bool ModelNode::attach(const ModelNodeAttachmentPtr& attachment)
    {
        WriteLockPtr w = getModel()->getWriteLock();
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

    bool ModelNode::isAttached(const ModelNodeAttachmentPtr& attachment)
    {
        ReadLockPtr r = getModel()->getReadLock();
        if (!attachment)
        {
            return false;
        }

        std::vector<ModelNodeAttachmentPtr> allWithType = attachments[attachment->getType()];
        return std::find(allWithType.begin(), allWithType.end(), attachment) != allWithType.end();
    }

    bool ModelNode::detach(const ModelNodeAttachmentPtr& attachment)
    {
        WriteLockPtr w = getModel()->getWriteLock();
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

    std::vector<ModelNodeAttachmentPtr> ModelNode::getAttachments(const std::string& type) const
    {
        ReadLockPtr r = getModel()->getReadLock();
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
            auto search = attachments.find(type);
            if (search != attachments.end())
            {
                return search->second;
            }
            else
            {
                return std::vector<ModelNodeAttachmentPtr>();
            }
        }
    }

    std::vector<ModelNodeAttachmentPtr> ModelNode::getAttachmentsWithVisualisation() const
    {
        ReadLockPtr r = getModel()->getReadLock();
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

    Eigen::Matrix4f ModelNode::getTransformationTo(const ModelNodePtr& otherObject)
    {
        return getGlobalPose().inverse() * otherObject->getGlobalPose();
    }

    Eigen::Matrix4f ModelNode::getTransformationFrom(const ModelNodePtr& otherObject)
    {
        return otherObject->getGlobalPose().inverse() * getGlobalPose();
    }
}
