#include "ModelNodeSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"

namespace VirtualRobot
{

    ModelNodeSet::ModelNodeSet(const std::string &name, 
        const ModelWeakPtr& model, 
        const std::vector<ModelNodePtr> &modelNodes,
        const ModelNodePtr kinematicRoot, 
        const CoordinatePtr tcp) :
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

    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(const ModelPtr& model, const std::string &name, const std::vector<std::string> &modelNodeNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        if (modelNodeNames.empty())
        {
            VR_WARNING << "Empty set of ModelNode names provided." << endl;
        }

        // model nodes
        std::vector<ModelNodePtr> modelNodes;
        modelNodes.reserve(modelNodeNames.size());
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

    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(const ModelPtr& model, 
        const std::string &name, 
        const std::vector<ModelNodePtr> &modelNodes, 
        const ModelNodePtr kinematicRoot, 
        const CoordinatePtr tcp, 
        bool registerToModel)
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
                THROW_VR_EXCEPTION_IF(model != modelNodes[i]->getModel(), "Model " + modelNodes[i]->getName() + " does not belong to the given model.");
            }

            // assert, that all model links have the same collision checker
            VirtualRobot::CollisionCheckerPtr collisionChecker;
            for (unsigned int i = 0; i < modelNodes.size(); i++)
            {
                if (ModelNode::checkNodeOfType(modelNodes[i], ModelNode::ModelNodeType::Link))
                {
                    ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
                    if (!collisionChecker)
                    {
                        collisionChecker = link->getCollisionChecker();
                    }
                    THROW_VR_EXCEPTION_IF(collisionChecker != link->getCollisionChecker(), "Model links belong to different collision checkers.");
                }
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
        CoordinatePtr tcpNode = tcp;
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

    std::vector<ModelNodePtr>::iterator ModelNodeSet::begin()
    {
        return modelNodes.begin();
    }

    std::vector<ModelNodePtr>::iterator ModelNodeSet::end()
    {
        return modelNodes.end();
    }

    bool ModelNodeSet::hasModelNode(const ModelNodePtr & node) const
    {
        return std::find(modelNodes.begin(), modelNodes.end(), node) != modelNodes.end();
    }

    bool ModelNodeSet::hasModelNode(const std::string &nodeName) const
    {
        for (const ModelNodePtr & node : modelNodes)
        {
            if (node->getName() == nodeName)
            {
                return true;
            }
        }
        return false;
    }

    const std::vector<ModelNodePtr> ModelNodeSet::getModelNodes() const
    {
        return modelNodes;
    }

    ModelNodePtr ModelNodeSet::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void ModelNodeSet::setKinematicRoot(const RobotNodePtr& modelNode)
    {
        this->kinematicRoot = modelNode;
    }

    CoordinatePtr ModelNodeSet::getTCP() const
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

        for (const ModelNodePtr & node : modelNodes)
        {
            cout << "--ModelNode Name: " << node->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
    }

    unsigned int ModelNodeSet::getSize() const
    {
        return modelNodes.size();
    }
    /*
    bool ModelNodeSet::isKinematicChain()
    {
        for (size_t i = modelNodes.size()-1; i > 0; --i)
        {
            ModelNodePtr tmpParent = modelNodes[i]->getParentNode(modelNodes[i-1]->getType());
            while (modelNodes[i-1] != tmpParent)
            {
                tmpParent = tmpParent->getParentNode(modelNodes[i-1]->getType());
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
    }*/

    bool ModelNodeSet::nodesSufficient(const std::vector<ModelNodePtr>& nodes) const
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

    ModelNodePtr ModelNodeSet::checkKinematicRoot(const std::string &kinematicRootName, ModelPtr model)
    {
        ModelNodePtr res;
        THROW_VR_EXCEPTION_IF (!model, "NULL model");
        if (!kinematicRootName.empty())
        {
            ModelNodePtr node = model->getModelNode(kinematicRootName);
            THROW_VR_EXCEPTION_IF(!node, "No root node with name " + kinematicRootName + " found.");
            res = node;
        }
        
        return res;
    }

    ModelNodePtr ModelNodeSet::checkTcp(const std::string &tcpName, ModelPtr model)
    {
        // tcp
        ModelNodePtr tcp;
        if (!tcpName.empty())
        {
            ModelNodePtr node = model->getModelNode(tcpName);
            THROW_VR_EXCEPTION_IF(!node, "No tcp node with name " + tcpName + " found.");
            tcp = node;
        }
        
        return tcp;
    }

    std::string ModelNodeSet::toXML(int tabs) const
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
            if (ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Joint))
            {
                ModelJointPtr modelJoint = std::static_pointer_cast<ModelJoint>(node);
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
            if (ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(node);
                modelLinks.push_back(link);
            }
        }
        return modelLinks;
    }
    
    std::vector<std::string> ModelNodeSet::getNodeNames() const
    {
        std::vector<std::string> res;
        for (auto n: modelNodes)
            res.push_back(n->getName());
        return res;
    }
}
