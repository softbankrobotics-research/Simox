#include "ModelNodeSet.h"
#include "VirtualRobot.h"
#include "Model.h"
#include "Nodes/ModelNode.h"
#include "VirtualRobotException.h"

namespace VirtualRobot
{

    ModelNodeSet::ModelNodeSet(const std::string &name, ModelWeakPtr model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const ModelNodePtr tcp)
    {
        this->name = name;
        this->model = model;
        this->modelNodes = modelNodes;
        this->kinematicRoot = kinematicRoot;
        this->tcp = tcp;

        if (modelNodes.size() > 0)
        {
            if (!kinematicRoot)
            {
                if (ModelPtr tmp = model.lock()) this->kinematicRoot = tmp->getRootNode();
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
        for (int i = 0; i < modelNodeNames.size(); i++)
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
            for (int i = 0; i < modelNodes.size(); i++)
            {
                THROW_VR_EXCEPTION_IF(modelNodes[0]->getModel() != modelNodes[i]->getModel(), "Model nodes belong to different models.");
            }

            // we collect all model links
            std::vector<ModelLinkPtr> modelLinks;
            for (int i = 0; i < modelNodes.size(); i++)
            {
                if (modelNodes[i]->getType() == ModelNode::ModelNodeType::Link)
                {
                    ModelLinkPtr link = boost::static_pointer_cast<ModelLink>(modelNodes[i]);
                    modelLinks.push_back(link);
                }
            }
            // assert, that all model links have the same collision checker
            for (int i = 0; i < modelLinks.size(); i++)
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
}
