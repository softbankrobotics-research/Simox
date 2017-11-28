#include "ModelNode.h"
#include "../Model.h"
#include "../../VirtualRobotException.h"
#include "../../Tools/ConditionedLock.h"
#include "../ModelNodeSet.h"
#include "../Frame.h"
#include "Attachments/ModelNodeAttachment.h"
#include "Attachments/Sensor.h"

namespace VirtualRobot
{
    ModelNode::ModelNode(const ModelWeakPtr& model, 
        const std::string& name,
        const Eigen::Matrix4f& staticTransformation)
        : Frame(name),
         // initialized(false),
          model(model),
          parent(ModelNodeWeakPtr()),
          children(),
          staticTransformation(staticTransformation),
          attachments()
    {
    }

    ModelNode::~ModelNode()
    {
    }

    ModelPtr ModelNode::getModel() const
    {
        ModelPtr modelShared = model.lock();
        if (!modelShared)
        {
            VR_INFO << "No model" << endl;
        }
        THROW_VR_EXCEPTION_IF(!modelShared, "ModelNode \"" + getName() + "\" does not have a model.");
        return modelShared;
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
        //THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

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
        //THROW_VR_EXCEPTION_IF(!isInitialized(), "ModelNode \"" + getName() + "\" is not initialized.");

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

    bool ModelNode::attachChild(const ModelNodePtr& newNode, bool updatePoses)
    {
		WriteLockPtr w = getModel()->getWriteLock();
		VR_ASSERT(newNode);

        if (this == newNode.get())
        {
            VR_WARNING << "Trying to attach a node to itself! name: " << getName() << std::endl;
            return false;
        }

        if (hasChild(newNode))
        {
            VR_WARNING << " Trying to attach an already attached node: " << getName() << "->"
                       << newNode->getName() << std::endl;
            return true; // no error
        }

        if (newNode->getParentNode())
        {
            VR_WARNING << " Trying to attach a node that already has a parent: " << getName() << "->"
                       << newNode->getName() << ", child's parent:" << newNode->getParentNode()->getName() << std::endl;
            return false;
        }

        if (hasChild(newNode->getName()))
        {
            VR_ERROR << " Trying to attach a node with name: " << newNode->getName() << " to "
                     << getName() << ", but a child with same name is already present!" << std::endl;
            return false;
        }

		if (getModel() != newNode->getModel())
		{
			VR_ERROR << " Trying to attach a node that belongs to a different model. New node: " << newNode->getName() << " Model: " << newNode->getModel()->getName() << std::endl;
			return false;
		}
		ModelPtr m = getModel();
		if (!m->hasModelNode(newNode))
			m->registerModelNode(newNode);
        children.push_back(newNode);
        newNode->parent = shared_from_this();

        if (updatePoses)
            updatePose(true);

		//initialized = true;

        return true;
    }

    bool ModelNode::hasChild(const ModelNodePtr& node, bool recursive) const
    {
        ReadLockPtr r = getModel()->getReadLock();

        // initialisation is checked in getChildNodes
        for (const ModelNodePtr & child : getChildNodes())
        {
            if (child == node)
            {
                return true;
            }
            if (recursive && child->hasChild(node, true))
            {
               return true;
            }
        }
        return false;
    }

    bool ModelNode::hasChild(const std::string& nodeName, bool recursive) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        // initialisation is checked in getChildNodes
        for (const ModelNodePtr & child : getChildNodes())
        {
            if (child->getName() == nodeName)
            {
                return true;
            }
            if (recursive && child->hasChild(nodeName, true))
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
			ModelPtr m = getModel();
			if (m->hasModelNode(node))
				m->deregisterModelNode(node);

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
                    attachment->update(globalPose);
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

    void ModelNode::updatePoseInternally(bool updateChildren, bool updateAttachments)
    {
    }

    bool ModelNode::attach(const ModelNodeAttachmentPtr& attachment)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        if (!attachment || isAttached(attachment) || !attachment->isAttachable(shared_from_this()) || hasAttachment(attachment->getName()))
        {
            return false;
        }

        // first setup the attachment
        attachment->setParent(shared_from_this());
        attachment->update(globalPose);

        attachments[attachment->getType()].push_back(attachment);

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


    bool ModelNode::hasAttachment(const std::string& attachmentName) const
	{
        return getAttachment(attachmentName).get() != 0;
	}

	ModelNodeAttachmentPtr ModelNode::getAttachment(const std::string& attachmentName) const
	{
		ReadLockPtr r = getModel()->getReadLock();
		for (auto a : attachments)
		{
			for (auto a2 : a.second)
			{
				if (a2->getName() == attachmentName)
					return a2;
			}
		}
		return ModelNodeAttachmentPtr();
	}

    bool ModelNode::detach(const ModelNodeAttachmentPtr& attachment)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        if (isAttached(attachment))
        {
            attachment->setParent(ModelNodePtr());
            attachments[attachment->getType()].erase(std::find(attachments[attachment->getType()].begin(), attachments[attachment->getType()].end(), attachment));

            return true;
        }

        return false;
    }

    bool ModelNode::detach(const std::string &attachmentName)
    {
        return detach(getAttachment(attachmentName));
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
        std::vector<ModelNodeAttachmentPtr> attachmentsWithVisualisation;
        for (auto it = attachments.begin(); it != attachments.end(); it++)
        {
            for (auto& attachment : it->second)
            {
                if (attachment->getVisualisation())
                {
                    attachmentsWithVisualisation.push_back(attachment);
                }
            }
        }
        return attachmentsWithVisualisation;
    }

    void ModelNode::print(bool printChildren, bool printDecoration) const
    {
        // TODO
    }

    /*std::string ModelNode::toXML(const std::string& basePath, const std::string& modelPathRelative, bool storeAttachments)
    {
        return std::string(); // TODO
    }*/

    ModelNodePtr ModelNode::clone(ModelPtr newModel, bool cloneChildren, bool cloneAttachments, ModelNodePtr parentNode, float scaling)
    {
        ReadLockPtr lock = getModel()->getReadLock();

        if (!newModel)
        {
            VR_ERROR << "Attempting to clone RobotNode for invalid robot";
            return ModelNodePtr();
        }

        //std::vector< std::string > clonedChildrenNames;

        //VisualizationNodePtr clonedVisualizationNode;

        /*if (Visualization)
        {
            clonedVisualizationNode = visualizationModel->clone(true, scaling);
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = collisionModel->clone(colChecker, scaling);
        }*/

        ModelNodePtr result = _clone(newModel, scaling);

        if (cloneAttachments)
        {
            for (auto &a : attachments)
            {
                for (auto &aa : a.second)
                {
                    ModelNodeAttachmentPtr ca = aa->clone();
                    if (ca)
                    {
                        result->attach(ca);
                    }
                }
            }

        }

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << endl;
            return result;
        }

        if (cloneChildren)
        {
            std::vector< ModelNodePtr > children = this->getChildNodes();

            for (auto n : children)
            {
                if (n)
                {
                    ModelNodePtr c = n->clone(newModel, true, true, ModelNodePtr(), scaling);

                    if (c)
                    {
                        result->attachChild(c, false);
                    }
                }
                /*else
                {
                    SensorPtr s =  dynamic_pointer_cast<Sensor>(children[i]);

                    if (s)
                    {
                        // performs registering and initialization
                        SensorPtr c = s->clone(result, scaling);
                    }
                    else
                    {
                        SceneObjectPtr so = children[i]->clone(children[i]->getName(), colChecker, scaling);

                        if (so)
                        {
                            result->attachChild(so);
                        }
                    }
                }*/
            }
        }

        /*result->setMaxVelocity(maxVelocity);
        result->setMaxAcceleration(maxAcceleration);
        result->setMaxTorque(maxTorque);
        result->setLimitless(limitless);

        std::map< std::string, float>::iterator it = propagatedJointValues.begin();

        while (it != propagatedJointValues.end())
        {
            result->propagateJointValue(it->first, it->second);
            it++;
        }*/

        if (parentNode)
        {
            parentNode->attachChild(result, false);
        }

        newModel->registerModelNode(result);

        newModel->applyJointValues();

        return result;
    }
}
