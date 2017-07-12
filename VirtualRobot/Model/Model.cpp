#include "Model.h"
#include "../VirtualRobotException.h"
#include "Nodes/ModelNode.h"
#include "Nodes/ModelLink.h"
#include "LinkSet.h"
#include "Nodes/ModelJoint.h"
#include "../Visualization/VisualizationNode.h"
#include "../CollisionDetection/CollisionModel.h"
#include "ModelConfig.h"
#include "../Trajectory.h"
#include "../EndEffector/EndEffector.h"

#include <algorithm>

namespace VirtualRobot
{
    Model::Model(const std::string& name, const std::string& type) : Coordinate(name), type(type), scaling(1.0f),
                                                                     threadsafe(true), mutex(),
                                                                     rootNode(),
                                                                     collisionChecker(),
                                                                     modelNodeMap(), 
                                                                     modelNodeSetMap(),
                                                                     filename("")
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
            THROW_VR_EXCEPTION_IF(!hasModelNode(node->getName()),
                                  "There are (at least) two model nodes with name <" + node->getName()
                                  + "> defined, the second one is skipped!");

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
            VR_WARNING << "No robot node with name <" << modelNodeName << "> registered." << endl;
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
            THROW_VR_EXCEPTION_IF(!hasModelNodeSet(nodeSet->getName()),
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
		return nodeSet && getModelNodeSet(nodeSet->getName()) == nodeSet;
	}

	bool Model::hasModelNodeSet(const std::string& name) const
	{
		ReadLockPtr r = getReadLock();
		return modelNodeSetMap.find(name) != modelNodeSetMap.end();
	}


	bool Model::hasJointSet(const JointSetPtr& nodeSet) const
	{
		return nodeSet && getJointSet(nodeSet->getName()) == nodeSet;
	}

	bool Model::hasJointSet(const std::string & name) const
	{
		ReadLockPtr r = getReadLock();
		auto mns = modelNodeSetMap.find(name);
		return (mns != modelNodeSetMap.end() && std::dynamic_pointer_cast<JointSet>(mns->second));
	}

	bool Model::hasLinkSet(const LinkSetPtr& nodeSet) const
	{
		return nodeSet && getLinkSet(nodeSet->getName()) == nodeSet;
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
           // VR_WARNING << "No link set with name <" << nodeSetName << "> registered." << endl;
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
            //VR_WARNING << "No joint set with name <" << nodeSetName << "> registered." << endl;
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
        std::string name = std::string("__" + name + "-all-links");
        std::shared_ptr<Model> m = shared_from_this();
        LinkSetPtr ls = LinkSet::createLinkSet(m, name, nodes);

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

    void Model::setRootNode(const ModelNodePtr& node)
    {
        if (!node)
        {
            return;
        }
        if (hasModelNode(node))
        {
            WriteLockPtr w = getWriteLock();
            rootNode = node;
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

    std::string Model::getName() const
    {
        return name;
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

    void Model::setGlobalPoseForModelNode(const CoordinatePtr& node, const Eigen::Matrix4f& globalPoseNode)
    {
        THROW_VR_EXCEPTION_IF(!node, "No node given.");
        THROW_VR_EXCEPTION_IF(!hasCoordinate(node), "Coordinate <" + node->getName() +
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
        rootNode->updatePose(true, true);
    }

    void Model::showStructure(bool enable)
    {
        // TODO: add Attachment
    }

    void Model::showCoordinateSystems(bool enable)
    {
        // TODO: add Attachment
    }

    /*void Model::highlight(const VisualizationPtr& visualization, bool enable)
    {
        // TODO: add Attachment
    }*/

    /*void Model::showPhysicsInformation(bool enableCoM, bool enableInertial, const VisualizationNodePtr& comModel)
    {
        // TODO: add Attachment
    }*/

    void Model::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(*iterator);
            link->getVisualization()->setupVisualization(showVisualization, showAttachedVisualizations);
        }
    }

    void Model::setUpdateVisualization(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(*iterator);
			if (link)
				link->getVisualization()->setUpdateVisualization(enable);
        }
    }

    void Model::setUpdateCollisionModel(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(*iterator);
			if (link)
				link->getCollisionModel()->setUpdateVisualization(enable);
        }
    }
	
	bool Model::getUpdateVisualization() const
	{
		WriteLockPtr w = getWriteLock();
		std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

		for (auto mn : modelNodes)
		{
			ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(mn);
			if (link)
				return link->getVisualization()->getUpdateVisualizationStatus();
		}
		return true;
	}

	bool Model::getUpdateCollisionModel() const
	{
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);
        for (auto mn : modelNodes)
		{
			ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(mn);
			if (link)
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

    void Model::setJointValues(const std::map<ModelNodePtr, float>& jointValues)
    {
        WriteLockPtr w = getWriteLock();

        for (auto it = jointValues.begin(); it != jointValues.end(); ++it)
        {
            THROW_VR_EXCEPTION_IF(!ModelNode::checkNodeOfType(it->first, ModelNode::ModelNodeType::Joint),
                                  "Can not set joint value of node <" + it->first->getName() + ">.");
            THROW_VR_EXCEPTION_IF(!hasModelNode(it->first), "Node <" + it->first->getName() + "> is not part of the model.");

            ModelJointPtr joint = std::static_pointer_cast<ModelJoint>(it->first);
            joint->setJointValueNoUpdate(it->second);
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
									float scaling)
    {
        ReadLockPtr r = getReadLock();
        THROW_VR_EXCEPTION_IF(!hasModelNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        CollisionCheckerPtr colChecker = collisionChecker;

        if (!colChecker)
        {
            colChecker = this->getCollisionChecker();
        }

        ModelPtr result(new Model(newModelName, newModelType));

        ModelNodePtr rootNew = startNode->clone(result, true, ModelNodePtr(), colChecker, scaling);
        THROW_VR_EXCEPTION_IF(!rootNew, "Clone failed...");
        result->setRootNode(rootNew);
        result->setScaling(scaling);

        std::vector<ModelNodePtr> rn = result->getModelNodes();

        // check for RNS that are covered by subpart
        if (cloneRNS)
        {
            for (auto it = modelNodeSetMap.begin(); it != modelNodeSetMap.end(); ++it)
            {
                if (it->second->nodesSufficient(rn))
                {
                    ModelNodeSetPtr rns = it->second->clone(result);

                    if (rns && !result->hasModelNodeSet(rns))
                    {
                        result->registerModelNodeSet(rns);
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
					EndEffectorPtr eef = it->second->clone(result);

					if (eef && !result->hasEndEffector(eef))
					{
						result->registerEndEffector(eef);
					}
				}
			}
		}


        std::vector<ModelNodePtr> allNodes;
        startNode->collectAllNodes(allNodes, ModelNode::ModelNodeType::Joint);

        for (size_t i = 0; i < allNodes.size(); i++)
        {
            ModelNodePtr roN = result->getModelNode(allNodes[i]->getName());

            if (roN)
            {
                std::static_pointer_cast<ModelJoint>(roN)->setJointValueNoUpdate(
                        std::static_pointer_cast<ModelJoint>(allNodes[i])->getJointValue());
            }
        }

        result->applyJointValues();
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

        return std::string(); // TODO: implement toXML
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

    ModelPtr Model::clone(const std::string& name, CollisionCheckerPtr collisionChecker, float scaling)
    {
        return VirtualRobot::ModelPtr(); // TODO: implement clone
    }
	/*
    Eigen::Matrix4f Model::toLocalCoordinateSystem(const Eigen::Matrix4f& poseGlobal) const
    {
        return getGlobalPose().inverse() * poseGlobal;
    }

    Eigen::Vector3f Model::toLocalCoordinateSystemVec(const Eigen::Vector3f& positionGlobal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionGlobal;
        t = toLocalCoordinateSystem(t);
        Eigen::Vector3f result = t.block(0, 3, 3, 1);
        return result;
    }

    Eigen::Matrix4f Model::toGlobalCoordinateSystem(const Eigen::Matrix4f& poseLocal) const
    {
        return getGlobalPose() * poseLocal;
    }

    Eigen::Vector3f Model::toGlobalCoordinateSystemVec(const Eigen::Vector3f& positionLocal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionLocal;
        t = toGlobalCoordinateSystem(t);
        Eigen::Vector3f result = t.block(0, 3, 3, 1);
        return result;
    }

	void Model::setName(const std::string &name)
	{
		this->name = name;
	}*/

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

	// todod
	bool Model::hasCoordinate(const CoordinatePtr & coord) const
	{
		return false;
	}
	bool VirtualRobot::Model::hasCoordinate(const std::string & coordinateName) const
	{
		return false;
	}
	CoordinatePtr VirtualRobot::Model::getCoordinate(const std::string & coordinateName) const
	{
		return CoordinatePtr();
	}
}
