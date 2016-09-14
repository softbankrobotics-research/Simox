#include "ModelNodeSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "VirtualRobotException.h"
#include "RobotConfig.h"
#include "KinematicChain.h"

namespace VirtualRobot
{

    ModelNodeSet::ModelNodeSet(const std::string &name, ModelWeakPtr model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const ModelNodePtr tcp) :
            name(name),
            weakModel(model),
            modelNodes(modelNodes),
            kinematicRoot(kinematicRoot),
            tcp(tcp)
    {
        if (modelNodes.size() > 0)
        {
            if (!kinematicRoot)
            {
                this->kinematicRoot = modelNodes.at(0);
            }
            if (!tcp)
            {
                    this->tcp = modelNodes.at(modelNodes.size() - 1);
            }
        }
        else
        {
            VR_WARNING << "Initializing empty ModelNodeSet." << endl;
        }
    }

    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(ModelPtr model, const std::string &name, const std::vector<std::string> &modelNodeNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        if (modelNodeNames.empty())
        {
            VR_WARNING << "Empty set of ModelNode names provided." << endl;
        }

        // model nodes
        std::vector<ModelNodePtr> modelNodes;
        for (size_t i = 0; i < modelNodeNames.size(); i++)
        {
            ModelNodePtr node = model->getModelNode(modelNodeNames[i]);
            THROW_VR_EXCEPTION_IF(!node, "No ModelNode with name " + modelNodeNames[i] + " found.");
            modelNodes.push_back(node);
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
        ModelNodePtr kinematicRoot;
        if (!kinematicRootName.empty())
        {
            ModelNodePtr node = model->getModelNode(kinematicRootName);
            THROW_VR_EXCEPTION_IF(!node, "No root node with name " + kinematicRootName + " found.");
            kinematicRoot = node;
        }
        else
        {
            if (!modelNodes.empty())
            {
                kinematicRoot = modelNodes[0];
            }
            else
            {
                THROW_VR_EXCEPTION("Can't determine a valid kinematic root node.");
            }
        }

        // tcp
        ModelNodePtr tcp;
        if (!tcpName.empty())
        {
            ModelNodePtr node = model->getModelNode(tcpName);
            THROW_VR_EXCEPTION_IF(!node, "No tcp node with name " + tcpName + " found.");
            tcp = node;
        }
        else
        {
            if (!modelNodes.empty())
            {
                tcp = modelNodes[modelNodes.size() - 1];
            }
            else
            {
                THROW_VR_EXCEPTION("Can't determine a valid tcp node.");
            }
        }

        ModelNodeSetPtr mns = createModelNodeSet(model, name, modelNodes, kinematicRoot, tcp, registerToModel);
        return mns;
    }

    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(ModelPtr model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const ModelNodePtr tcp, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        if (modelNodes.empty())
        {
            VR_WARNING << "Empty set of ModelNodes provided." << endl;
        }
        else
        {
            for (size_t i = 0; i < modelNodes.size(); i++)
            {
                THROW_VR_EXCEPTION_IF(modelNodes[0]->getModel() != modelNodes[i]->getModel(), "Model nodes belong to different models.");
            }

            // we collect all model links
            std::vector<ModelLinkPtr> modelLinks;
            for (size_t i = 0; i < modelNodes.size(); i++)
            {
                if (modelNodes[i]->getType() == ModelNode::ModelNodeType::Link)
                {
                    ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(modelNodes[i]);
                    modelLinks.push_back(link);
                }
            }

            // assert, that all model links have the same collision checker
            for (size_t i = 0; i < modelLinks.size(); i++)
            {
                THROW_VR_EXCEPTION_IF(modelLinks[0]->getCollisionChecker() != modelLinks[i]->getCollisionChecker(), "Model links belong to different collision checkers.");
            }
        }

        // kinematic root
        // we do not check whether the given kinematic root is actually a root node
        ModelNodePtr kinematicRootNode = kinematicRoot;
        if (!kinematicRootNode)
        {
            if (!modelNodes.empty())
            {
                kinematicRootNode = modelNodes[0];
            }
            else
            {
                THROW_VR_EXCEPTION("Can't determine a valid kinematic root node.");
            }
        }

        // tcp
        ModelNodePtr tcpNode = tcp;
        if (!tcpNode)
        {
            if (!modelNodes.empty())
            {
                tcpNode = modelNodes[modelNodes.size() - 1];
            }
            else
            {
                THROW_VR_EXCEPTION("Can't determine a valid tcp node.");
            }
        }

        ModelNodeSetPtr mns(new ModelNodeSet(name, model, modelNodes, kinematicRootNode, tcpNode));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasModelNodeSet(mns), "ModelNodeSet with name " + name + " already present in the model");
            model->registerModelNodeSet(mns);
        }

        return mns;
    }

    std::string ModelNodeSet::getName() const
    {
        return name;
    }

    ModelPtr ModelNodeSet::getModel() const
    {
        return weakModel.lock();
    }

    ModelNodePtr &ModelNodeSet::getNode(int i)
    {
        THROW_VR_EXCEPTION_IF((i >= (int)modelNodes.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (modelNodes.size() - 1));
        return modelNodes.at(i);
    }

    std::vector::iterator ModelNodeSet::begin()
    {
        return modelNodes.begin();
    }

    std::vector::iterator ModelNodeSet::end()
    {
        return modelNodes.end();
    }

    bool ModelNodeSet::hasModelNode(ModelNodePtr node) const
    {
        return std::find(modelNodes.begin(), modelNodes.end(), node) != modelNodes.end();
    }

    bool ModelNodeSet::hasModelNode(const std::string &nodeName) const
    {
        for (ModelNodePtr node : modelNodes)
        {
            if (node->getName() == nodeName)
            {
                return true;
            }
        }
        return false;
    }

    const std::vector<ModelNodePtr> ModelNodeSet::getAllModelNodes() const
    {
        return modelNodes;
    }

    ModelNodePtr ModelNodeSet::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void ModelNodeSet::setKinematicRoot(RobotNodePtr modelNode)
    {
        this->kinematicRoot = modelNode;
    }

    ModelNodePtr ModelNodeSet::getTCP() const
    {
        return tcp;
    }

    void ModelNodeSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "ModelNodeSet:" << endl;
        std::cout << "Name: " << name << endl;

        std::string modelName = "-";
        if (ModelPtr model = weakModel.lock())
        {
            modelName = model->getName();
        }

        std::cout << "Model Name: " << modelName << endl;
        std::cout << "Kinematic root: " << kinematicRoot->getName() << endl;
        std::cout << "TCP: " << tcp->getName() << endl;
        std::cout << "ModelNodes:" << endl;

        for (ModelNodePtr node : modelNodes)
        {
            cout << "--ModelNode Name: " << node->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
    }

    unsigned int ModelNodeSet::getSize() const
    {
        return modelNodes.size();
    }

    std::vector<CollisionModelPtr> ModelNodeSet::getCollisionModels()
    {
        std::vector<CollisionModelPtr> collisionModels;

        // we collect all model links
        std::vector<ModelLinkPtr> modelLinks;
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            if (modelNodes[i]->getType() == ModelNode::ModelNodeType::Link)
            {
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(modelNodes[i]);
                modelLinks.push_back(link);
            }
        }

        // we retrieve the collisionmodels from the model links
        for (ModelLinkPtr link : modelLinks)
        {
            collisionModels.push_back(link->getCollisionModel());
        }

        return collisionModels;
    }

    std::vector<float> ModelNodeSet::getJointValues() const
    {
        std::vector<float> fillVector;
        getJointValues(fillVector);

        return fillVector;
    }

    void ModelNodeSet::getJointValues(std::vector<float> &fillVector, bool clearVector) const
    {
        if (clearVector)
        {
            fillVector.clear();
        }

        std::vector<ModelJointPtr> modelJoints = getModelJoints();

        for (ModelJointPtr joint : modelJoints)
        {
            fillVector.push_back(joint->getJointValue());
        }
    }

    void ModelNodeSet::getJointValues(Eigen::VectorXf &fillVector) const
    {
        std::vector<float> jointValues = getJointValues();
        fillVector.resize(jointValues.size());

        for (size_t i = 0; i < jointValues.size(); i++)
        {
            fillVector[i] = jointValues[i];
        }
    }

    // TODO Adapt RobotConfig to v3 before implementing this method.
    void ModelNodeSet::getJointValues(RobotConfigPtr config) const
    {
        ;
    }

    void ModelNodeSet::respectJointLimits(std::vector<float>& jointValues) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    void ModelNodeSet::respectJointLimits(Eigen::VectorXf& jointValues) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    bool ModelNodeSet::checkJointLimits(std::vector<float>& jointValues, bool verbose) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        bool ret = true;
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            if (!modelJoints[i]->checkJointLimits(jointValues[i], verbose))
            {
                ret = false;
            }
        }
        return ret;
    }

    bool ModelNodeSet::checkJointLimits(Eigen::VectorXf& jointValues, bool verbose) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        bool result = true;
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            if (!modelJoints[i]->checkJointLimits(jointValues[i], verbose))
            {
                result = false;
            }
        }
        return result;
    }

    void ModelNodeSet::setJointValues(const std::vector<float>& jointValues)
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        ModelPtr model = weakModel.lock();
        VR_ASSERT(model);
        WriteLockPtr lock = model->getWriteLock();

        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->setJointValueNoUpdate(jointValues[i]);
        }

        if (kinematicRoot)
        {
            kinematicRoot->updatePose();
        }
        else
        {
            model->applyJointValues();
        }
    }

    void ModelNodeSet::setJointValues(const Eigen::VectorXf& jointValues)
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        ModelPtr model = weakModel.lock();
        VR_ASSERT(model);
        WriteLockPtr lock = model->getWriteLock();

        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->setJointValueNoUpdate(jointValues[i]);
        }

        if (kinematicRoot)
        {
            kinematicRoot->updatePose();
        }
        else
        {
            model->applyJointValues();
        }
    }

    // TODO Adapt RobotConfig to v3 before implementing this method.
    void ModelNodeSet::setJointValues(const RobotConfigPtr config)
    {
        ;
    }

    bool ModelNodeSet::isKinematicChain()
    {
        for (size_t i = 0; i < this->modelNodes.size() - 1; i++)
        {
            ModelNodePtr tmpParent = modelNodes[i + 1]->getParentNode();
            while (modelNodes[i] != tmpParent)
            {
                tmpParent = tmpParent->getParentNode();
                if (!tmpParent)
                {
                    return false;
                }
            }
        }

        return true;
    }

    // TODO there are many plausible strategies to do that, for now just return an empty pointer
    KinematicChainPtr ModelNodeSet::toKinematicChain()
    {
        return KinematicChainPtr();
    }

    int ModelNodeSet::getNumFaces(bool collisionModel)
    {
        int result = 0;
        std::vector<ModelLinkPtr> modelLinks = getModelLinks();

        for (ModelLinkPtr link : modelLinks)
        {
            result += link->getNumFaces(collisionModel);
        }

        return result;
    }

    float ModelNodeSet::getMaximumExtension()
    {
        float result = 0;
        Eigen::Matrix4f t;
        Eigen::Vector3f v;

        if (kinematicRoot && modelNodes.size() > 0)
        {
            t = kinematicRoot->getTransformationTo(modelNodes[0]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        for (size_t i = 0; i < modelNodes.size() - 1; i++)
        {
            t = modelNodes[i]->getTransformationTo(modelNodes[i + 1]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        if (tcp && modelNodes.size() > 0)
        {
            t = tcp->getTransformationTo(modelNodes[modelNodes.size() - 1]);
            v = MathTools::getTranslation(t);
            result += v.norm();
        }

        return result;
    }

    Eigen::Vector3f ModelNodeSet::getCoM()
    {
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        std::vector<ModelLinkPtr> modelLinks = getModelLinks();
        for (size_t i = 0; i < modelLinks.size(); i++)
        {
            res += modelLinks[i]->getCoMGlobal() * modelLinks[i]->getMass() / m;
        }

        return res;
    }

    float ModelNodeSet::getMass()
    {
        float result = 0;
        std::vector<ModelLinkPtr> modelLinks = getModelLinks();

        for (ModelLinkPtr link : modelLinks)
        {
            result += link->getMass();
        }

        return result;
    }

    bool ModelNodeSet::nodesSufficient(std::vector<ModelNodePtr> nodes) const
    {
        bool tcpOk = false;
        bool krOk = false;

        if (!tcp)
        {
            tcpOk = true;
        }

        if (!kinematicRoot)
        {
            krOk = true;
        }

        std::vector<ModelNodePtr>::const_iterator j = nodes.begin();
        bool ok = false;

        while (j != nodes.end())
        {
            if (!tcpOk && (*j)->getName() == tcp->getName())
            {
                tcpOk = true;
            }

            if (!krOk && (*j)->getName() == kinematicRoot->getName())
            {
                krOk = true;
            }

            j++;
        }

        if (!krOk || !tcpOk)
        {
            return false;
        }

        std::vector<ModelNodePtr>::const_iterator i = modelNodes.begin();

        while (i != modelNodes.end())
        {
            std::vector<ModelNodePtr>::const_iterator j = nodes.begin();
            bool ok = false;

            while (j != nodes.end())
            {
                if ((*i)->getName() == (*j)->getName())
                {
                    ok = true;
                    break;
                }

                j++;
            }

            if (!ok)
            {
                return false;
            }

            i++;
        }

        return true;
    }

    std::string ModelNodeSet::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<RobotNodeSet name='" << name << "'>\n";

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ss << pre << t << "<Node name='" << modelNodes[i]->getName() << "'/>\n";
        }

        ss << pre << "</RobotNodeSet>\n";
        return ss.str();
    }
    
    std::vector<ModelJointPtr> ModelNodeSet::getModelJoints() const
    {
        std::vector<ModelJointPtr> modelJoints;
        for (ModelNodePtr node : modelNodes)
        {
            if (node->getType() & ModelNode::ModelNodeType::Joint)
            {
                ModelJointPtr modelJoint = boost::static_pointer_cast<ModelJoint>(node);
                modelJoints.push_back(modelJoint);
            }
        }
        return modelJoints;
    }

    std::vector<ModelLinkPtr> ModelNodeSet::getModelLinks() const
    {
        std::vector<ModelLinkPtr> modelLinks;
        for (ModelNodePtr node : modelNodes)
        {
            if (node->getType() == ModelNode::ModelNodeType::Link)
            {
                ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(node);
                modelLinks.push_back(link);
            }
        }
        return modelLinks;
    }
}
