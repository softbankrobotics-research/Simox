#include "ModelNodeSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"
#include "Nodes/Attachments/ModelNodeAttachment.h"

namespace VirtualRobot
{
    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(const ModelPtr &model, const std::string &name, const std::vector<std::string> &modelNodeNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        // model nodes
        std::vector<ModelNodePtr> modelNodes;
        modelNodes.reserve(modelNodeNames.size());
        for (const auto& nodeName : modelNodeNames)
        {
            ModelNodePtr node = model->getNode(nodeName);
            THROW_VR_EXCEPTION_IF(!node, "No ModelNode with name " + nodeName + " found.");
            modelNodes.push_back(node);
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
        ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);

        //tcp
        FramePtr tcp = checkTcp(tcpName, model);

        ModelNodeSetPtr mns = createModelNodeSet(model, name, modelNodes, kinematicRoot, tcp, registerToModel);
        return mns;
    }


    ModelNodeSetPtr ModelNodeSet::createModelNodeSet(const ModelPtr &model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr &kinematicRoot, const FramePtr &tcp, bool registerToModel)
    {
        ModelNodeSetPtr mns(new Implementation(name, model, modelNodes, kinematicRoot, tcp));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasNodeSet(mns), "NodeSet with name " + name + " already present in the model");
            model->registerNodeSet(mns);
        }

        return mns;
    }


    ModelNodeSet::ModelNodeSet(const std::string &name, const ModelWeakPtr &model, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr &kinematicRoot, const FramePtr &tcp) :
        name(name),
        weakModel(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "Model not initialized.");

        if (modelNodes.size() == 0)
        {
            VR_WARNING << "Initializing empty Set." << endl;
        }
        else
        {
            VirtualRobot::CollisionCheckerPtr collisionChecker = nullptr;
            for (const auto& node : modelNodes)
            {
                THROW_VR_EXCEPTION_IF(model.lock() != node->getModel(), "Model " + node->getName() + " does not belong to the given model.");

                // assert, that all model links have the same collision checker
                if (ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Link))
                {
                    ModelLinkPtr link = std::static_pointer_cast<ModelLink>(node);
                    if (!collisionChecker)
                    {
                        collisionChecker = link->getCollisionChecker();
                    }
                    THROW_VR_EXCEPTION_IF(collisionChecker != link->getCollisionChecker(), "Model links belong to different collision checkers.");
                }
            }
        }
    }


    ModelNodeSet::~ModelNodeSet()
    {
    }


    std::string ModelNodeSet::getName() const
    {
        return name;
    }


    ModelPtr ModelNodeSet::getModel() const
    {
        auto m = weakModel.lock();
        THROW_VR_EXCEPTION_IF(!m, "No model exists for set with name: " + getName());
        return m;
    }

    std::vector<std::string> ModelNodeSet::getNodeNames() const
    {
        std::vector<std::string> names;
        names.reserve(getSize());
        for (const auto& node : getNodes())
        {
            names.push_back(node->getName());
        }
        return names;
    }

    bool ModelNodeSet::nodesSufficient(const std::vector<ModelNodePtr> &nodes) const
    {
        bool tcpOk = false;
        bool krOk = false;

        auto tcp = getTCP();
        auto kinematicRoot = getKinematicRoot();

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

        for (const auto& n1 : getNodes())
        {
            bool ok = false;
            for (const auto& n2 : nodes)
            {
                if (n1->getName() == n2->getName())
                {
                    ok = true;
                    break;
                }
            }
            if (!ok)
            {
                return false;
            }
        }

        return true;
    }


    ModelNodePtr ModelNodeSet::checkKinematicRoot(const std::string &name, const ModelPtr &model)
    {
        VR_ASSERT(model);
        if (!name.empty())
        {
            ModelNodePtr node = model->getNode(name);
            THROW_VR_EXCEPTION_IF(!node, "No root node with name " + name + " found.");
            return node;
        }

        return nullptr;
    }


    FramePtr ModelNodeSet::checkTcp(const std::string &tcpName, const ModelPtr &model)
    {
        VR_ASSERT(model);
        if (!tcpName.empty())
        {
            FramePtr f = model->getNode(tcpName);
            if (!f)
            {
                f = model->getAttachment(tcpName);
            }
            THROW_VR_EXCEPTION_IF(!f, "No tcp node with name " + tcpName + " found.");
            return f;
        }
        return nullptr;
    }

    ModelNodeSet::Implementation::Implementation(const std::string &name, const ModelWeakPtr &model, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr& kinematicRoot, const FramePtr& tcp) :
        ModelNodeSet(name, model, modelNodes, kinematicRoot, tcp),
        modelNodes(modelNodes),
        kinematicRoot(kinematicRoot)
    {
        if (!modelNodes.empty())
        {
            if (!tcp)
            {
                this->tcp = modelNodes.at(modelNodes.size()-1);
            }

            if (!kinematicRoot)
            {
                this->kinematicRoot = modelNodes.at(0);
            }
        }
    }

    ModelNodePtr ModelNodeSet::Implementation::getNode(size_t i) const
    {
        return modelNodes.at(i);
    }

    bool ModelNodeSet::Implementation::hasNode(const ModelNodePtr &node) const
    {
        return std::find(modelNodes.begin(), modelNodes.end(), node) != modelNodes.end();
    }

    bool ModelNodeSet::Implementation::hasNode(const std::string &nodeName) const
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

    std::vector<ModelNodePtr> ModelNodeSet::Implementation::getNodes() const
    {
        return modelNodes;
    }

    std::vector<ModelJointPtr> ModelNodeSet::Implementation::getJoints() const
    {
        std::vector<ModelJointPtr> modelJoints;
        for (ModelNodePtr node : getNodes())
        {
            if (ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Joint))
            {
                ModelJointPtr modelJoint = std::static_pointer_cast<ModelJoint>(node);
                modelJoints.push_back(modelJoint);
            }
        }
        return modelJoints;
    }

    std::vector<ModelLinkPtr> ModelNodeSet::Implementation::getLinks() const
    {
        std::vector<ModelLinkPtr> modelLinks;
        for (ModelNodePtr node : getNodes())
        {
            if (ModelNode::checkNodeOfType(node, ModelNode::ModelNodeType::Link))
            {
                ModelLinkPtr link = std::static_pointer_cast<ModelLink>(node);
                modelLinks.push_back(link);
            }
        }
        return modelLinks;
    }

    unsigned int ModelNodeSet::Implementation::getSize() const
    {
        return getNodes().size();
    }

    ModelNodePtr ModelNodeSet::Implementation::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void ModelNodeSet::Implementation::setKinematicRoot(const ModelNodePtr &modelNode)
    {
        THROW_VR_EXCEPTION_IF(!modelNode, "New kinematicRoot does not exist.");
        this->kinematicRoot = modelNode;
    }

    FramePtr ModelNodeSet::Implementation::getTCP() const
    {
        return tcp;
    }

    void ModelNodeSet::Implementation::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "ModelNodeSet:" << endl;
        std::cout << "Name: " << getName() << endl;

        std::string modelName = "-";
        if (ModelPtr model = getModel())
        {
            modelName = model->getName();
        }

        std::cout << "Model Name: " << modelName << endl;
        std::cout << "Kinematic root: " << getKinematicRoot()->getName() << endl;
        std::cout << "TCP: " << getTCP()->getName() << endl;
        std::cout << "ModelNodes:" << endl;

        for (const ModelNodePtr & node : getNodes())
        {
            cout << "--ModelNode Name: " << node->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
    }

    std::string ModelNodeSet::Implementation::toXML(int tabs) const
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<RobotNodeSet name='" << getName() << "'>\n";

        for (const auto& node : getNodes())
        {
            ss << pre << t << "<Node name='" << node->getName() << "'/>\n";
        }

        ss << pre << "</RobotNodeSet>\n";
        return ss.str();
    }

    ModelNodeSetPtr ModelNodeSet::Implementation::clone(const ModelPtr &model, const std::string &newName, bool registerToModel) const
    {
        std::vector<ModelNodePtr> newModelNodes;
        for (const auto &n : getNodes())
        {
            THROW_VR_EXCEPTION_IF(!model->hasNode(n->getName()), "Cannot clone, new model does not contain node " << n->getName());
            ModelNodePtr newModelNode = model->getNode(n->getName());
            VR_ASSERT(newModelNode);
            newModelNodes.push_back(newModelNode);
        }
        ModelNodePtr newKinRoot = nullptr;
        if (getKinematicRoot())
        {
            newKinRoot = checkKinematicRoot(getKinematicRoot()->getName(), model);
            VR_ASSERT(newKinRoot);
        }
        FramePtr newTcp = nullptr;
        if (getTCP())
        {
            newTcp = checkTcp(getTCP()->getName(), model);
            VR_ASSERT(newTcp);
        }

        ModelNodeSetPtr result = ModelNodeSet::createModelNodeSet(model, (newName.empty() ? getName() : newName), newModelNodes, newKinRoot, newTcp, registerToModel);

        return result;
    }
}
