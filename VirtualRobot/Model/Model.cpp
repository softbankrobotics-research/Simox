#include "Model.h"
#include "../VirtualRobotException.h"
#include "Nodes/ModelNode.h"
#include "Nodes/ModelLink.h"
#include "LinkSet.h"
#include "Nodes/ModelJoint.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/VisualizationSet.h"
#include "../CollisionDetection/CollisionModel.h"
#include "ModelConfig.h"
#include "../Trajectory.h"
#include "../EndEffector/EndEffector.h"
#include "../Model/Nodes/Attachments/ModelNodeAttachment.h"
#include "../Model/Nodes/Attachments/ModelNodeAttachmentFactory.h"
#include "../Model/Nodes/Attachments/ModelStructureFactory.h"
#include "../Model/Nodes/Attachments/PhysicsAttachment.h"
#include "../Model/Nodes/Attachments/PhysicsAttachmentFactory.h"
#include "../Model/Nodes/Attachments/Sensor.h"

#include <algorithm>

namespace VirtualRobot
{
    Model::Model(const std::string& name, const std::string& type) : Frame(name), type(type), scaling(1.0f),
                                                                     threadsafe(true), mutex(),
                                                                     rootNode(),
                                                                     collisionChecker(),
                                                                     modelNodeMap(), 
                                                                     modelNodeSetMap(),
                                                                     filename(""),
                                                                     visualizationNodeSetFull(VisualizationFactory::getInstance()->createVisualisationSet({})),
                                                                     visualizationNodeSetCollision(VisualizationFactory::getInstance()->createVisualisationSet({}))
    {
    }

    Model::~Model()
    {
    }

    void Model::registerModelNode(const ModelNodePtr& node)
    {
        WriteLockPtr w = getWriteLock();
        if (node)
        {
            if (hasModelNode(node->getName()))
            {
                if (node == getModelNode(node->getName()))
                    return;
                THROW_VR_EXCEPTION_IF(hasModelNode(node->getName()),
                                      "There are (at least) two model nodes with name <" + node->getName()
                                      + "> defined!");
            }

            if(ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(node);
                if (collisionChecker)
                {
                    THROW_VR_EXCEPTION_IF(link->getCollisionChecker() != collisionChecker,
                                          "Collision checker of node <" + node->getName()
                                          + "> does not match the collision checker of other nodes of model <"
                                          + getName() + ">.");
                }
                else
                {
                    collisionChecker = link->getCollisionChecker();
                }
                addToVisualization(link);
            }

            modelNodeMap[node->getName()] = node;
        }
    }

    void Model::deregisterModelNode(const ModelNodePtr& node)
    {
        WriteLockPtr w = getWriteLock();
        if (node)
        {
            auto i = modelNodeMap.find(node->getName());

            if (i != modelNodeMap.end())
            {
                modelNodeMap.erase(i);
            }
        }
        if(ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Link))
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(node);
            removeFromVisualization(link);
        }
    }

    bool Model::hasModelNode(const ModelNodePtr& node) const
    {
        return node && getModelNode(node->getName()) == node;
    }

    bool Model::hasModelNode(const std::string& modelNodeName) const
    {
        ReadLockPtr r = getReadLock();
        return (modelNodeMap.find(modelNodeName) != modelNodeMap.end());
    }

    bool Model::hasJoint(const std::string& jointName) const
    {
        ReadLockPtr r = getReadLock();
        auto i = modelNodeMap.find(jointName);
        return (i != modelNodeMap.end() && (i->second->isJoint()));
    }

    bool Model::hasLink(const std::string& linkName) const
    {
        ReadLockPtr r = getReadLock();
        auto i = modelNodeMap.find(linkName);
        return (i != modelNodeMap.end() && (i->second->isLink()));
    }

	bool Model::hasLink(const ModelLinkPtr& link) const
	{
		ReadLockPtr r = getReadLock();
		if (!link)
			return false;
		auto i = modelNodeMap.find(link->getName());
		return (i != modelNodeMap.end() && (i->second->isLink()) && i->second == link);
	}

	bool Model::hasJoint(const ModelJointPtr& joint) const
	{
		ReadLockPtr r = getReadLock();
		if (!joint)
			return false;
		auto i = modelNodeMap.find(joint->getName());
		return (i != modelNodeMap.end() && (i->second->isJoint()) && i->second == joint);
	}

    ModelNodePtr Model::getModelNode(const std::string& modelNodeName) const
    {
        ReadLockPtr r = getReadLock();
        auto search = modelNodeMap.find(modelNodeName);
        if (search == modelNodeMap.end())
        {
            //VR_WARNING << "No robot node with name <" << modelNodeName << "> registered." << endl;
            return RobotNodePtr();
        }

        return search->second;
    }

    ModelLinkPtr Model::getLink(const std::string& modelNodeName) const
    {
        ReadLockPtr r = getReadLock();
        auto search = modelNodeMap.find(modelNodeName);
        if (search == modelNodeMap.end())
        {
            VR_WARNING << "No robot node with name <" << modelNodeName << "> registered." << endl;
            return ModelLinkPtr();
        }

        ModelLinkPtr res = std::dynamic_pointer_cast<ModelLink>(search->second);
        if (!res)
        {
            VR_WARNING << "Model node with name <" << modelNodeName << "> is registered, but not a link." << endl;
        }
        return res;
    }

    ModelJointPtr Model::getJoint(const std::string& modelNodeName) const
    {
        ReadLockPtr r = getReadLock();
        auto search = modelNodeMap.find(modelNodeName);
        if (search == modelNodeMap.end())
        {
            VR_WARNING << "No robot node with name <" << modelNodeName << "> registered." << endl;
            return ModelJointPtr();
        }

        ModelJointPtr res = std::dynamic_pointer_cast<ModelJoint>(search->second);
        if (!res)
        {
            VR_WARNING << "Model node with name <" << modelNodeName << "> is registered, but not a joint." << endl;
        }
        return res;
    }

    std::vector<ModelNodePtr> Model::getModelNodes(ModelNode::ModelNodeType type) const
    {
        ReadLockPtr r = getReadLock();
        std::vector<ModelNodePtr> result;
        getModelNodes(result, false, type);
        return result;
    }

    std::vector<ModelNodePtr> Model::getModelNodes(const std::vector<std::string> &nodeNames) const
    {
        std::vector<ModelNodePtr> modelNodes;
        modelNodes.reserve(nodeNames.size());
        for (size_t i = 0; i < nodeNames.size(); i++)
        {
            ModelNodePtr node = getModelNode(nodeNames[i]);
            THROW_VR_EXCEPTION_IF(!node, "No ModelNode with name " + nodeNames[i] + " found.");
            modelNodes.push_back(node);
        }

        return modelNodes;
    }

    void Model::getModelNodes(std::vector<ModelNodePtr>& storeNodes, bool clearVector, ModelNode::ModelNodeType type) const
    {
        ReadLockPtr r = getReadLock();
        if (clearVector)
        {
            storeNodes.clear();
        }

        storeNodes.reserve(storeNodes.size() + modelNodeMap.size());

        for(auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, type))
            {
                storeNodes.push_back(it->second);
            }
        }
    }

    void Model::registerModelNodeSet(const ModelNodeSetPtr& nodeSet)
    {
        WriteLockPtr w = getWriteLock();
        if (nodeSet)
        {
            THROW_VR_EXCEPTION_IF(hasModelNodeSet(nodeSet->getName()),
                                  "There are (at least) two model nodes with name <" + nodeSet->getName()
                                  + "> defined, the second one is skipped!");
            modelNodeSetMap[nodeSet->getName()] = nodeSet;
        }
    }

	void VirtualRobot::Model::registerJointSet(const JointSetPtr & nodeSet)
	{
		registerModelNodeSet(nodeSet);
	}

	void VirtualRobot::Model::registerLinkSet(const LinkSetPtr & nodeSet)
	{
		registerModelNodeSet(nodeSet);
	}

    void Model::deregisterModelNodeSet(const ModelNodeSetPtr& nodeSet)
    {
        WriteLockPtr w = getWriteLock();
        if (nodeSet)
        {
            std::string nodeSetName = nodeSet->getName();
            auto i = modelNodeSetMap.find(nodeSetName);

            if (i != modelNodeSetMap.end())
            {
                modelNodeSetMap.erase(i);
            }
        }
    }

	void VirtualRobot::Model::deregisterJointSet(const JointSetPtr & nodeSet)
	{
		deregisterModelNodeSet(nodeSet);
	}

	void VirtualRobot::Model::deregisterLinkSet(const LinkSetPtr & nodeSet)
	{
		deregisterModelNodeSet(nodeSet);
	}


	bool Model::hasModelNodeSet(const ModelNodeSetPtr& nodeSet) const
	{
        return nodeSet && hasModelNodeSet(nodeSet->getName());
	}

	bool Model::hasModelNodeSet(const std::string& name) const
	{
		ReadLockPtr r = getReadLock();
		return modelNodeSetMap.find(name) != modelNodeSetMap.end();
	}


	bool Model::hasJointSet(const JointSetPtr& nodeSet) const
	{
        return nodeSet && hasJointSet(nodeSet->getName());
	}

	bool Model::hasJointSet(const std::string & name) const
	{
		ReadLockPtr r = getReadLock();
		auto mns = modelNodeSetMap.find(name);
		return (mns != modelNodeSetMap.end() && std::dynamic_pointer_cast<JointSet>(mns->second));
	}

	bool Model::hasLinkSet(const LinkSetPtr& nodeSet) const
	{
        return nodeSet && hasLinkSet(nodeSet->getName());
	}

	bool Model::hasLinkSet(const std::string & name) const
	{
		ReadLockPtr r = getReadLock();
		auto mns = modelNodeSetMap.find(name);
		return (mns != modelNodeSetMap.end() && std::dynamic_pointer_cast<LinkSet>(mns->second));
	}

    ModelNodeSetPtr Model::getModelNodeSet(const std::string& nodeSetName) const
    {
        ReadLockPtr r = getReadLock();
        auto search = modelNodeSetMap.find(nodeSetName);
        if (search == modelNodeSetMap.end())
        {
            //VR_WARNING << "No robot node set with name <" << nodeSetName << "> registered." << endl;
            return RobotNodeSetPtr();
        }

        return search->second;
    }

    LinkSetPtr Model::getLinkSet(const std::string& nodeSetName) const
    {
        ReadLockPtr r = getReadLock();
        ModelNodeSetPtr res = getModelNodeSet(nodeSetName);
        LinkSetPtr ls = std::dynamic_pointer_cast<LinkSet>(res);
        if (!ls)
        {
            VR_WARNING << "No link set with name <" << nodeSetName << "> registered." << endl;
        }
        return ls;
    }

    JointSetPtr Model::getJointSet(const std::string& nodeSetName) const
    {
        ReadLockPtr r = getReadLock();
        ModelNodeSetPtr res = getModelNodeSet(nodeSetName);
        JointSetPtr ls = std::dynamic_pointer_cast<JointSet>(res);
        if (!ls)
        {
            VR_WARNING << "No joint set with name <" << nodeSetName << "> registered." << endl;
        }
        return ls;
    }

    std::vector<ModelLinkPtr> Model::getLinks() const
    {
        ReadLockPtr r = getReadLock();
        std::vector< ModelLinkPtr > nodes;
        for (auto n : modelNodeMap)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n.second);
            if (l)
                nodes.push_back(l);
        }
        return nodes;
    }

    LinkSetPtr Model::getLinkSet()
    {
        ReadLockPtr r = getReadLock();
        std::vector< ModelNodePtr > nodes;
        for (auto n : modelNodeMap)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(n.second);
            if (l)
                nodes.push_back(n.second);
        }
        std::string nameLS = std::string("__") + name + std::string("-all-links");
        std::shared_ptr<Model> m = shared_from_this();
        LinkSetPtr ls = LinkSet::createLinkSet(m, nameLS, nodes);

        return ls;
    }

    std::vector<ModelJointPtr> Model::getJoints() const
    {
        ReadLockPtr r = getReadLock();
        std::vector< ModelJointPtr > nodes;
        for (auto n : modelNodeMap)
        {
            ModelJointPtr l = std::dynamic_pointer_cast<ModelJoint>(n.second);
            if (l)
                nodes.push_back(l);
        }
        return nodes;
    }

    JointSetPtr Model::getJointSet()
    {
        ReadLockPtr r = getReadLock();
        std::vector< ModelNodePtr > nodes;
        for (auto n : modelNodeMap)
        {
            ModelJointPtr l = std::dynamic_pointer_cast<ModelJoint>(n.second);
            if (l)
                nodes.push_back(n.second);
        }
        std::string name = std::string("__" + name + "-all-joints");
        ModelPtr m = shared_from_this();
        JointSetPtr ls = JointSet::createJointSet(m, name, nodes);

        return ls;
    }

    std::vector<ModelNodeSetPtr> Model::getModelNodeSets() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<ModelNodeSetPtr> result;
        for (auto it : modelNodeSetMap)
        {
            result.push_back(it.second);
        }
        return result;
    }

    std::vector<LinkSetPtr> Model::getLinkSets() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<LinkSetPtr> result;
        for (auto it : modelNodeSetMap)
        {
            LinkSetPtr ls = std::dynamic_pointer_cast<LinkSet>(it.second);
            if (ls)
                result.push_back(ls);
        }
        return result;
    }

    std::vector<JointSetPtr> Model::getJointSets() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<JointSetPtr> result;
        for (auto it : modelNodeSetMap)
        {
            JointSetPtr ls = std::dynamic_pointer_cast<JointSet>(it.second);
            if (ls)
                result.push_back(ls);
        }
        return result;
    }

    void Model::setRootNode(const ModelNodePtr& node, bool updatePose)
    {
        if (!node)
        {
            return;
        }
        if (hasModelNode(node))
        {
            WriteLockPtr w = getWriteLock();
            rootNode = node;
            if (updatePose)
                rootNode->updatePose(true, true);
        }
        else
        {
            THROW_VR_EXCEPTION("Root node must be registered at the model.");
        }
    }

	ModelNodePtr Model::getRootNode() const
	{
		ReadLockPtr r = getReadLock();
		return rootNode;
	}

	ModelLinkPtr Model::getFirstLink() const
	{
		ReadLockPtr r = getReadLock();
		if (!rootNode)
			return ModelLinkPtr();
		if (rootNode->isLink())
			return std::dynamic_pointer_cast<ModelLink>(rootNode);
		auto n = rootNode->getChildNodes(ModelNode::Link);
		if (n.size()==0)
			return ModelLinkPtr();
		return std::dynamic_pointer_cast<ModelLink>(n.at(0));
	}

	std::string Model::getType() const
    {
        return type;
    }

    void Model::setThreadsafe(bool flag)
    {
        threadsafe = flag;
    }

    void Model::setGlobalPose(const Eigen::Matrix4f& globalPose, bool applyValues)
    {
        WriteLockPtr w = getWriteLock();
        this->globalPose = globalPose;

        if (applyValues)
        {
            rootNode->updatePose(true, true);
        }
    }

    Eigen::Matrix4f Model::getGlobalPose() const
    {
        ReadLockPtr r = getReadLock();
        return globalPose;
    }

    Eigen::Vector3f Model::getGlobalPosition() const
    {
        ReadLockPtr r = getReadLock();
        return globalPose.block<3, 1>(0, 3);
    }

    void Model::setGlobalPoseForModelNode(const FramePtr& node, const Eigen::Matrix4f& globalPoseNode)
    {
        THROW_VR_EXCEPTION_IF(!node, "No node given.");
        THROW_VR_EXCEPTION_IF(!hasFrame(node), "Frame <" + node->getName() +
                                                   "> is not part of model <" + getName() + ">");

        // get transformation from current to wanted tcp pose
        Eigen::Matrix4f t = globalPoseNode * node->getGlobalPose().inverse();

        // apply transformation to current global pose of robot
        t = t * getGlobalPose();

        // set t
        setGlobalPose(t, true);
    }

    Eigen::Vector3f Model::getCoMLocal() const
    {
        return toLocalCoordinateSystemVec(getCoMGlobal());
    }

    Eigen::Vector3f Model::getCoMGlobal() const
    {
        ReadLockPtr r = getReadLock();
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(*iterator);
            res += link->getCoMGlobal() * link->getMass() / m;
        }

        return res;
    }

    float Model::getScaling() const
    {
        ReadLockPtr r = getReadLock();
        return scaling;
    }

    void Model::setScaling(float scaling)
    {
        WriteLockPtr e = getWriteLock();
        this->scaling = scaling;
    }

    void Model::applyJointValues()
    {
        if (rootNode)
            rootNode->updatePose(true, true);
    }

    void Model::attachStructure()
    {
        ModelNodeAttachmentFactoryPtr attachmentFactory = ModelNodeAttachmentFactory::fromName(ModelStructureFactory::getName(), nullptr);
        for (const auto & node : getModelNodes())
        {
            std::string attachmentName = node->getName() + "_structure";
            ModelNodeAttachmentPtr attachment = attachmentFactory->createAttachment(attachmentName, Eigen::Matrix4f::Identity());
            node->attach(attachment);
        }
    }

    void Model::detachStructure()
    {
        for (const auto & node : getModelNodes())
        {
            node->detach(node->getName() + "_structure");
        }
    }

    void Model::attachFrames()
    {
        ModelNodeAttachmentFactoryPtr attachmentFactory = ModelNodeAttachmentFactory::fromName(ModelNodeAttachmentFactory::getName(), nullptr);
        for (const auto & joint : getJoints())
        {
            std::string attachmentName = joint->getName() + "_frame";
            ModelNodeAttachmentPtr attachment = attachmentFactory->createAttachment(attachmentName, Eigen::Matrix4f::Identity());
            joint->attach(attachment);
        }
    }

    void Model::detachFrames()
    {
        for (const auto & joint : getJoints())
        {
            joint->detach(joint->getName() + "_frame");
        }
    }

    void Model::attachPhysicsInformation()
    {
        ModelNodeAttachmentFactoryPtr attachmentFactory = ModelNodeAttachmentFactory::fromName(PhysicsAttachmentFactory::getName(), nullptr);
        for (auto &&link : getLinks())
        {
            if (link->getMass() <= 0)
                continue;

            std::string attachmentName = link->getName() + "_physics";
            ModelNodeAttachmentPtr attachment = attachmentFactory->createAttachment(attachmentName, Eigen::Matrix4f::Identity());
            link->attach(attachment);
        }
    }

    void Model::detachPhysicsInformation()
    {
        for (auto &&link : getLinks())
        {
            link->detach(link->getName() + "_physics");
        }
    }

    void Model::setupVisualization(bool showVisualization)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(*iterator);
            if (link && link->getVisualization())
                link->getVisualization()->setVisible(showVisualization);
            else if (link)
            {
                // todo setup visualization
            }
        }
    }

    void Model::setUpdateVisualization(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        getVisualization(VirtualRobot::ModelLink::Full)->setUpdateVisualization(enable);
    }

    void Model::setUpdateCollisionModel(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(*iterator);
            if (link && link->getCollisionModel())
				link->getCollisionModel()->setUpdateVisualization(enable);
        }
    }

    bool Model::getUpdateVisualization() const
    {
        WriteLockPtr w = getWriteLock();
        return getVisualization(VirtualRobot::ModelLink::Full)->getUpdateVisualizationStatus();
    }

	bool Model::getUpdateCollisionModel() const
	{
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);
        for (auto mn : modelNodes)
		{
			ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(mn);
            if (link && link->getCollisionModel() )
				return link->getCollisionModel()->getUpdateVisualizationStatus();
		}
		return true;
	}

	ModelConfigPtr Model::getConfig()
    {
        ReadLockPtr rl = getReadLock();
        ModelConfigPtr r(new ModelConfig(shared_from_this(), getName()));

        std::vector<ModelNodePtr> modelNodes = this->getModelNodes();
        std::vector<ModelNodePtr>::const_iterator iterator = modelNodes.begin();

        while (modelNodes.end() != iterator)
        {
            ModelNodePtr rn = *iterator;

            if (ModelNode::checkNodeOfType(rn, ModelNode::ModelNodeType::JointPrismatic)
                ||ModelNode::checkNodeOfType(rn, ModelNode::ModelNodeType::JointRevolute))
            {
                ModelJointPtr joint = std::static_pointer_cast<ModelJoint>(rn);
                r->setConfig(joint, joint->getJointValue());
            }

            iterator++;
        }

        return r;
    }

    bool Model::setConfig(const ModelConfigPtr& c)
    {
        if (!c)
        {
            return false;
        }

        setJointValues(c);
        return true;
    }

    void Model::setJointValue(const ModelNodePtr& node, float jointValue)
    {
        if (node)
        {
            THROW_VR_EXCEPTION_IF(!ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Joint),
                                  "Can not set joint value of node <" + node->getName() + ">.");
            THROW_VR_EXCEPTION_IF(!hasModelNode(node), "Node <" + node->getName() + "> is not part of the model.");

            ModelJointPtr joint = std::static_pointer_cast<ModelJoint>(node);
            joint->setJointValue(jointValue);
        }
    }

    void Model::setJointValue(const std::string& nodeName, float jointValue)
    {
        THROW_VR_EXCEPTION_IF(!hasModelNode(nodeName), "Model does not have a node with name <" + nodeName + ">.");
        setJointValue(getModelNode(nodeName), jointValue);
    }

    void Model::setJointValues(const std::map<std::string, float>& jointValues)
    {
        WriteLockPtr w = getWriteLock();

        for (auto it = jointValues.begin(); it != jointValues.end(); ++it)
        {
            THROW_VR_EXCEPTION_IF(!hasModelNode(it->first), "Node <" + it->first + "> is not part of the model.");

            ModelNodePtr node = getModelNode(it->first);
            THROW_VR_EXCEPTION_IF(!ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Joint),
                                  "Can not set joint value of node <" + node->getName() + ">.");

            ModelJointPtr joint = std::static_pointer_cast<ModelJoint>(node);
            joint->setJointValueNoUpdate(it->second);
        }

        applyJointValues();
    }

    void Model::setJointValues(const std::map<ModelJointPtr, float> &jointValues)
    {
        WriteLockPtr w = getWriteLock();

        for (auto it = jointValues.begin(); it != jointValues.end(); ++it)
        {
            THROW_VR_EXCEPTION_IF(!hasModelNode(it->first), "Node <" + it->first->getName() + "> is not part of the model.");
            (it->first)->setJointValueNoUpdate(it->second);
        }
        applyJointValues();
    }

    void Model::setJointValues(const ModelConfigPtr& config)
    {
        if (config)
        {
            config->setJointValues(shared_from_this());
        }
    }

    void Model::setJointValues(const TrajectoryPtr& trajectory, float t)
    {
        if (trajectory)
        {
            Eigen::VectorXf c;
            trajectory->interpolate(t, c);
            trajectory->getJointSet()->setJointValues(c);
        }
    }

    int Model::getNumFaces(bool collisionModel)
    {
        ReadLockPtr r = getReadLock();
        int res = 0;

        for (auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(it->second);
                res += link->getNumFaces(collisionModel);
            }
        }

        return res;
    }

    BoundingBox Model::getBoundingBox(bool collisionModel)
    {
        ReadLockPtr r = getReadLock();
        VirtualRobot::BoundingBox bbox;

        for (auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(it->second);
                if (collisionModel && link->getCollisionModel())
                {
                    bbox.addPoints(link->getCollisionModel()->getBoundingBox());
                }
                else if (!collisionModel && link->getVisualization())
                {
                    bbox.addPoints(link->getVisualization()->getBoundingBox());
                }
            }
        }

        return bbox;
    }

    std::vector<CollisionModelPtr> Model::getCollisionModels() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<CollisionModelPtr> result;
        result.reserve(modelNodeMap.size());

        for (auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(it->second);
                if (link && link->getCollisionModel())
                    result.push_back(link->getCollisionModel());
            }
        }

        return result;
    }

    CollisionCheckerPtr Model::getCollisionChecker() const
    {
        ReadLockPtr r = getReadLock();
        return collisionChecker;
    }

    float Model::getMass() const
    {
        ReadLockPtr r = getReadLock();
        float mass = 0;

        for (auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(it->second);
                mass += link->getMass();
            }
        }

        return mass;
    }

    ModelPtr Model::extractSubPart(const ModelNodePtr& startNode,
									const std::string& newModelType, 
									const std::string& newModelName,
									bool cloneRNS,
									bool cloneEEF,
									const CollisionCheckerPtr& collisionChecker,
                                    float scaling) const
    {
        ReadLockPtr r = getReadLock();
        THROW_VR_EXCEPTION_IF(!hasModelNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        ModelPtr result(new Model(newModelName, newModelType));
        _clone(result, startNode, collisionChecker, cloneRNS, cloneEEF, scaling);
        return result;
    }

    void Model::attachNodeTo(const ModelNodePtr& newNode, const ModelNodePtr& existingNode)
    {
        WriteLockPtr w = getWriteLock();
        THROW_VR_EXCEPTION_IF(!hasModelNode(existingNode), "Model does not have node <" + existingNode->getName() + ">.");
        if (!hasModelNode(newNode))
        {
            registerModelNode(newNode);
        }
        existingNode->attachChild(newNode);
    }

    void Model::attachNodeTo(const ModelNodePtr& newNode, const std::string& existingNodeName)
    {
        WriteLockPtr w = getWriteLock();
        THROW_VR_EXCEPTION_IF(!hasModelNode(existingNodeName), "Model does not have node <" + existingNodeName + ">.");
        attachNodeTo(newNode, getModelNode(existingNodeName));
    }

    void Model::detachNode(const ModelNodePtr& node)
    {
        WriteLockPtr w = getWriteLock();
        if (hasModelNode(node))
        {
            ModelNodePtr parent = node->getParentNode();

            std::vector<ModelNodePtr> toDetach;
            node->collectAllNodes(toDetach);

            parent->detachChild(node);

            for (ModelNodePtr n : toDetach)
            {
                deregisterModelNode(n);
            }
        }
    }

    void Model::detachNode(const std::string& nodeName)
    {
        ReadLockPtr r = getReadLock();
        if (hasModelNode(nodeName))
        {
            detachNode(getModelNode(nodeName));
        }
    }

    void Model::setFilename(const std::string& filename)
    {
        WriteLockPtr w = getWriteLock();
        this->filename = filename;
    }

    std::string Model::getFilename() const
    {
        ReadLockPtr r = getReadLock();
        return filename;
    }

    std::string Model::toXML(const std::string& basePath, const std::string& modelPath,
                             bool storeEEF, bool storeRNS, bool storeAttachments)
    {
        std::stringstream ss;
        //ss << "<?xml version='1.0' encoding='UTF-8'?>" << endl << endl;
        ss << "<Model Type='" << this->type << "' RootNode='" << this->getRootNode()->getName() << "'>" << endl << endl;
        std::vector<ModelNodePtr> nodes = getModelNodes();

        for (size_t i = 0; i < nodes.size(); i++)
        {
            ss << nodes[i]->toXML(basePath, modelPath, storeAttachments) << endl;
        }

        ss << endl;

        if (storeRNS)
        {
            std::vector<RobotNodeSetPtr> rns = this->getModelNodeSets();

            for (size_t i = 0; i < rns.size(); i++)
            {
                ss << rns[i]->toXML(1) << endl;
            }

            if (rns.size() > 0)
            {
                ss << endl;
            }
        }

        if (storeEEF)
        {
            std::vector<EndEffectorPtr> eefs = this->getEndEffectors();

            for (size_t i = 0; i < eefs.size(); i++)
            {
                ss << eefs[i]->toXML(1) << endl;
            }

            if (eefs.size() > 0)
            {
                ss << endl;
            }
        }

        ss << "</Model>" << endl;

        return ss.str();
    }

    void Model::print()
    {
        // TODO: implement print
    }

    ReadLockPtr Model::getReadLock() const
    {
        if (!threadsafe)
        {
            return ReadLockPtr();
        }
        else
        {
            return ReadLockPtr(new ReadLock(mutex, threadsafe));
        }
    }

    WriteLockPtr Model::getWriteLock() const
    {
        if (!threadsafe)
        {
            return WriteLockPtr();
        }
        else
        {
            return WriteLockPtr(new WriteLock(mutex, threadsafe));
        }
    }

    ModelPtr Model::clone(const std::string& name, CollisionCheckerPtr collisionChecker, float scaling) const
    {
        VirtualRobot::ModelPtr result = extractSubPart(this->getRootNode(), this->getType(), name, true, true, collisionChecker, scaling);
        result->filename = filename;
        result->type = type;
        return result;
    }
 

	bool Model::hasEndEffector(const EndEffectorPtr &eef) const
	{
		ReadLockPtr w = getReadLock();
		if (!eef)
			return false;
		return hasEndEffector(eef->getName());
		return false;
	}

	bool Model::hasEndEffector(const std::string &name) const
	{
		ReadLockPtr w = getReadLock();
		auto i = eefMap.find(name);
		return (i != eefMap.end());
	}


	void Model::registerEndEffector(const EndEffectorPtr &eef)
    {
        VR_ASSERT(eef);
        WriteLockPtr w = getWriteLock();
        if (hasEndEffector(eef->getName()))
        {
            THROW_VR_EXCEPTION("EEF with name " + eef->getName() + " already registered to robot " + name);
        }
        eefMap[eef->getName()] = eef;
    }

    void Model::deregisterEndEffector(const EndEffectorPtr &eef)
    {
        WriteLockPtr w = getWriteLock();
        if (eef)
        {
            auto i = eefMap.find(eef->getName());

            if (i != eefMap.end())
            {
                eefMap.erase(i);
            }
        }
    }

    EndEffectorPtr Model::getEndEffector(const std::string & name) const
    {
        ReadLockPtr r = getReadLock();
        auto i = eefMap.find(name);
        if (i == eefMap.end())
            return EndEffectorPtr();

        return i->second;
    }

    std::vector<EndEffectorPtr> Model::getEndEffectors() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<EndEffectorPtr> res;

        for (auto it : eefMap)
        {
            res.push_back(it.second);
        }
        return res;
    }

    VisualizationSetPtr Model::getVisualization(ModelLink::VisualizationType visuType) const
    {
        return visuType == ModelLink::VisualizationType::Full ? visualizationNodeSetFull : visualizationNodeSetCollision;
    }

    VisualizationGroupPtr Model::getAllAttachmentVisualizations() const
    {
        VisualizationGroupPtr ret(new VisualizationGroup);
        for (const auto & node : getModelNodes())
        {
            for (const auto & attachement : node->getAttachmentsWithVisualisation())
            {
                VisualizationPtr v = attachement->getVisualisation();
                if (!v)
                    continue;

                ret->addVisualization(v);
            }
        }
        return ret;
    }

    void Model::addToVisualization(const ModelLinkPtr &link)
    {
        auto visuFull = link->getVisualization(ModelLink::VisualizationType::Full);
        if (visuFull)
        {
            visualizationNodeSetFull->addVisualization(visuFull);
        }
        auto visuCollision = link->getVisualization(ModelLink::VisualizationType::Collision);
        if (visuCollision)
        {
            visualizationNodeSetCollision->addVisualization(visuCollision);
        }
    }

    void Model::removeFromVisualization(const ModelLinkPtr &link)
    {
        auto visuFull = link->getVisualization(ModelLink::VisualizationType::Full);
        if (visuFull)
        {
            visualizationNodeSetFull->removeVisualization(visuFull);
        }
        auto visuCollision = link->getVisualization(ModelLink::VisualizationType::Collision);
        if (visuCollision)
        {
            visualizationNodeSetCollision->removeVisualization(visuCollision);
        }
    }

    void Model::_clone(ModelPtr newModel, const ModelNodePtr &startNode, const CollisionCheckerPtr &collisionChecker, bool cloneRNS, bool cloneEEF, float scaling) const
    {
        ReadLockPtr r = getReadLock();
        VR_ASSERT(newModel);
        VR_ASSERT(startNode);
        THROW_VR_EXCEPTION_IF(!hasModelNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        CollisionCheckerPtr colChecker = collisionChecker;

        if (!colChecker)
        {
            colChecker = this->getCollisionChecker();
        }

        ModelNodePtr rootNew = startNode->clone(newModel, true, true, ModelNodePtr(), scaling); // This also clones (and registers) all children to the new model.
        THROW_VR_EXCEPTION_IF(!rootNew, "Cloning failed...");
        newModel->setRootNode(rootNew);
        newModel->setScaling(scaling);

        std::vector<ModelNodePtr> rn = newModel->getModelNodes();

        // check for RNS that are covered by subpart
        if (cloneRNS)
        {
            for (auto it = modelNodeSetMap.begin(); it != modelNodeSetMap.end(); ++it)
            {
                if (it->second->nodesSufficient(rn))
                {
                    JointSetPtr js = std::dynamic_pointer_cast<JointSet>(it->second);
                    if (js)
                    {
                        JointSetPtr rns = js->clone(newModel);

                        if (rns && !newModel->hasJointSet(rns))
                        {
                            newModel->registerJointSet(rns);
                        }
                        continue;
                    }
                    LinkSetPtr ls = std::dynamic_pointer_cast<LinkSet>(it->second);
                    if (ls)
                    {
                        LinkSetPtr rns = ls->clone(newModel);
                        if (rns && !newModel->hasLinkSet(rns))
                        {
                            newModel->registerLinkSet(rns);
                        }
                        continue;
                    }

                    ModelNodeSetPtr rns = it->second->clone(newModel);
                    if (rns && !newModel->hasModelNodeSet(rns))
                    {
                        newModel->registerModelNodeSet(rns);
                    }
                }
            }
        }

        // check for EEF that are covered by subpart
        if (cloneEEF)
        {
            for (auto it = eefMap.begin(); it != eefMap.end(); ++it)
            {
                if (it->second->nodesSufficient(rn))
                {
                    EndEffectorPtr eef = it->second->clone(newModel);

                    if (eef && !newModel->hasEndEffector(eef))
                    {
                        newModel->registerEndEffector(eef);
                    }
                }
            }
        }


        std::vector<ModelNodePtr> allNodes;
        startNode->collectAllNodes(allNodes, ModelNode::ModelNodeType::Joint);

        for (size_t i = 0; i < allNodes.size(); i++)
        {
            ModelNodePtr roN = newModel->getModelNode(allNodes[i]->getName());

            if (roN)
            {
                std::static_pointer_cast<ModelJoint>(roN)->setJointValueNoUpdate(
                        std::static_pointer_cast<ModelJoint>(allNodes[i])->getJointValue());
            }
        }

        newModel->applyJointValues();
        newModel->setGlobalPose(getGlobalPose());
    }

	bool Model::hasFrame(const FramePtr & frame) const
	{
		if (!frame)
			return false;
		FramePtr f = getFrame(frame->getName());
		return f == frame;
	}

	bool Model::hasFrame(const std::string & frameName) const
	{
		FramePtr f = getFrame(frameName);
		return f.get()!=0;
	}

	FramePtr Model::getFrame(const std::string & frameName) const
	{
		ReadLockPtr r = getReadLock();

		// check model nodes and attachments
		for (auto n : modelNodeMap)
		{
			if (n.second->getName() == frameName)
				return n.second;

			ModelNodeAttachmentPtr a = n.second->getAttachment(frameName);
			if (a)
			{
				FramePtr f = std::dynamic_pointer_cast<Frame>(a);
				return f;
			}

		}
		return FramePtr();
	}

    void Model::registerConfiguration(const ModelConfigPtr &config)
    {
        WriteLockPtr w = getWriteLock();
        THROW_VR_EXCEPTION_IF(!config, "NULL config data");

        if (hasConfiguration(config))
        {
            return;
        }

        configs.push_back(config);
    }

    void Model::registerConfiguration(const std::vector<ModelConfigPtr> &configs)
    {
        WriteLockPtr w = getWriteLock();
        for (size_t i = 0; i < configs.size(); i++)
        {
            registerConfiguration(configs[i]);
        }
    }

    void Model::deRegisterConfiguration(const ModelConfigPtr &config)
    {
        WriteLockPtr w = getWriteLock();
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        if (!hasConfiguration(config))
        {
            return;
        }

        for (std::vector< ModelConfigPtr >::iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i) == config)
            {
                configs.erase(i);
                break;
            }
        }
    }

    void Model::deRegisterConfiguration(const std::string& name)
    {
        WriteLockPtr w = getWriteLock();
        if (!hasConfiguration(name))
        {
            return;
        }

        for (std::vector< ModelConfigPtr >::iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                configs.erase(i);
                break;
            }
        }
    }

    bool Model::hasConfiguration(const ModelConfigPtr &config) const
    {
        ReadLockPtr r = getReadLock();
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        for (std::vector< ModelConfigPtr >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            if (*i == config)
            {
                return true;
            }
        }

        return false;
    }

    bool Model::hasConfiguration(const std::string& name) const
    {
        ReadLockPtr r = getReadLock();
        for (std::vector< ModelConfigPtr >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return true;
            }
        }

        return false;
    }


    VirtualRobot::ModelConfigPtr Model::getConfiguration(const std::string& name) const
    {
        ReadLockPtr r = getReadLock();
        for (std::vector< ModelConfigPtr >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No Configuration with name " << name << " registered for model " << getName() << endl;
        return ModelConfigPtr();
    }

    std::vector<ModelConfigPtr> Model::getConfigurations() const
    {
        ReadLockPtr r = getReadLock();
        return configs;
    }

    std::vector<SensorPtr> Model::getSensors() const
    {
        return getAttachments<Sensor>();
    }

    bool Model::hasSensor(const std::string &name) const
    {
        std::vector<SensorPtr> sensors = getSensors();
        for (auto &s : sensors)
        {
            if (s && s->getName() == name)
                return true;
        }
        return false;
    }

    SensorPtr Model::getSensor(const std::string &name) const
    {
        std::vector<SensorPtr> sensors = getSensors();
        for (auto &s : sensors)
        {
            if (s && s->getName() == name)
                return s;
        }
        return SensorPtr();
    }


    std::vector<ModelNodeAttachmentPtr> Model::getAttachments() const
    {
        return getAttachments<ModelNodeAttachment>();
    }

    bool Model::hasAttachment(const std::string &name) const
    {
        std::vector<ModelNodeAttachmentPtr> att = getAttachments();
        for (auto &a : att)
        {
            if (a && a->getName() == name)
                return true;
        }
        return false;
    }

    ModelNodeAttachmentPtr Model::getAttachment(const std::string &name) const
    {
        std::vector<ModelNodeAttachmentPtr> att = getAttachments();
        for (auto &a : att)
        {
            if (a && a->getName() == name)
                return a;
        }
        return ModelNodeAttachmentPtr();
    }

}
