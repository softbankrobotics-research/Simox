#include "Model.h"
#include "VirtualRobotException.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "Visualization/VisualizationNode.h"
#include "CollisionDetection/CollisionModel.h"
#include "ModelConfig.h"
#include "Trajectory.h"

#include <algorithm>

namespace VirtualRobot
{
    Model::Model(const std::string& name, const std::string& type) : name(name), type(type), scaling(1.0f),
                                                                     threadsafe(true), mutex(),
                                                                     rootNode(),
                                                                     collisionChecker(),
                                                                     modelNodeMap(), modelNodeSetMap(),
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
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(node);
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

    std::vector<ModelNodePtr> Model::getModelNodes(ModelNode::ModelNodeType type) const
    {
        ReadLockPtr r = getReadLock();
        std::vector<ModelNodePtr> result;
        getModelNodes(result, false, type);
        return result;
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

    bool Model::hasModelNodeSet(const ModelNodeSetPtr& nodeSet) const
    {
        return nodeSet && getModelNodeSet(nodeSet->getName()) == nodeSet;
    }

    bool Model::hasModelNodeSet(const std::string& name) const
    {
        ReadLockPtr r = getReadLock();
        return modelNodeSetMap.find(name) != modelNodeSetMap.end();
    }

    ModelNodeSetPtr Model::getModelNodeSet(const std::string& nodeSetName) const
    {
        ReadLockPtr r = getReadLock();
        auto search = modelNodeSetMap.find(nodeSetName);
        if (search == modelNodeSetName.end())
        {
            VR_WARNING << "No robot node set with name <" << nodeSetName << "> registered." << endl;
            return RobotNodeSetPtr();
        }

        return search->second;
    }

    std::vector<ModelNodeSetPtr> Model::getModelNodeSets() const
    {
        ReadLockPtr r = getReadLock();
        std::vector<ModelNodeSetPtr> result;
        getModelNodeSets(result, false);
        return result;
    }

    void Model::getModelNodeSets(std::vector<ModelNodeSetPtr>& storeNodeSet, bool clearVector) const
    {
        ReadLockPtr r = getReadLock();
        if (clearVector)
        {
            storeNodeSet.clear();
        }

        storeNodeSet.reserve(storeNodeSet.size() + modelNodeSetMap.size());

        for(auto it = modelNodeSetMap.begin(); it != modelNodeSetMap.end(); ++it)
        {
            storeNodeSet.push_back(it->second);
        }
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

    void Model::setGlobalPoseForModelNode(const ModelNodePtr& node, const Eigen::Matrix4f& globalPoseNode)
    {
        THROW_VR_EXCEPTION_IF(!node, "No node given.");
        THROW_VR_EXCEPTION_IF(!hasModelNode(node), "Node <" + node->getName() +
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
            ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(*iterator);
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

    void Model::highlight(const VisualizationPtr& visualization, bool enable)
    {
        // TODO: add Attachment
    }

    void Model::showPhysicsInformation(bool enableCoM, bool enableInertial, const VisualizationNodePtr& comModel)
    {
        // TODO: add Attachment
    }

    void Model::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(*iterator);
            link->getVisualization()->setupVisualization(showVisualization, showAttachedVisualizations);
        }
    }

    void Model::setUpdateVisualization(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(*iterator);
            link->getVisualization()->setUpdateVisualization(enable);
        }
    }

    void Model::setUpdateCollisionModel(bool enable)
    {
        WriteLockPtr w = getWriteLock();
        std::vector<ModelNodePtr> modelNodes = this->getModelNodes(ModelNode::ModelNodeType::Link);

        for (auto iterator = modelNodes.begin(); modelNodes.end() != iterator; ++ iterator)
        {
            ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(*iterator);
            link->getCollisionModel()->setUpdateVisualization(enable);
        }
    }

    ModelConfigPtr Model::getConfig()
    {
        ReadLockPtr r = getReadLock();
        ModelConfigPtr r(new ModelConfig(shared_from_this(), getName()));

        std::vector<ModelNodePtr> modelNodes = this->getModelNodes();
        std::vector<ModelNodePtr>::const_iterator iterator = modelNodes.begin();

        while (modelNodes.end() != iterator)
        {
            ModelNodePtr rn = *iterator;

            if (ModelNode::checkNodeOfType(rn, ModelNode::ModelNodeType::JointPrismatic)
                ||ModelNode::checkNodeOfType(rn, ModelNode::ModelNodeType::JointRevolute))
            {
                ModelJointPtr joint = boost::static_pointer_cast<ModelJoint>(rn);
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

            ModelJointPtr joint = boost::static_pointer_cast<ModelJoint>(node);
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

            ModelJointPtr joint = boost::static_pointer_cast<ModelJoint>(node);
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

            ModelJointPtr joint = boost::static_pointer_cast<ModelJoint>(it->first);
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
            trajectory->getRobotNodeSet()->setJointValues(c);
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
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(it->second);
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
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(it->second);
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
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(it->second);
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
        int mass = 0;

        for (auto it = modelNodeMap.begin(); it != modelNodeMap.end(); ++it)
        {
            if (ModelNode::checkNodeOfType(it->second, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(it->second);
                mass += link->getMass();
            }
        }

        return mass;
    }

    ModelPtr Model::extractSubPart(const ModelNodePtr& startNode,
                                   const std::string& newModelType, const std::string& newModelName,
                                   bool cloneRNS, const CollisionCheckerPtr& collisionChecker, float scaling)
    {
        ReadLockPtr r = getReadLock();
        THROW_VR_EXCEPTION_IF(!hasModelNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        CollisionCheckerPtr colChecker = collisionChecker;

        if (!colChecker)
        {
            colChecker = this->getCollisionChecker();
        }

        //stefan Warning!!!!! which robot-type to create
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

        std::vector<ModelNodePtr> allNodes;
        startNode->collectAllNodes(allNodes, ModelNode::ModelNodeType::Joint);

        for (size_t i = 0; i < allNodes.size(); i++)
        {
            ModelNodePtr roN = result->getModelNode(allNodes[i]->getName());

            if (roN)
            {
                boost::static_pointer_cast<ModelJoint>(roN)->setJointValueNoUpdate(
                        boost::static_pointer_cast<ModelJoint>(allNodes[i])->getJointValue());
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
}
