#include "LinkSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "VirtualRobotException.h"
#include "ModelConfig.h"
#include "KinematicChain.h"

namespace VirtualRobot
{

    LinkSet::LinkSet(const std::string &name, const ModelWeakPtr& model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const ModelNodePtr tcp) :
            ModelNodeSet(name, model, modelNodes, kinematicRoot, tcp)
    {
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr j = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            THROW_VR_EXCEPTION_IF(!j, "Only links allowed in joint sets.");
            links.push_back(j);
        }
    }

    LinkSetPtr LinkSet::createLinkSet(const std::string &name, const ModelPtr& model, const std::vector<std::string> &modelNodeNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        if (modelNodeNames.empty())
        {
            VR_WARNING << "Empty set of ModelNode names provided." << endl;
        }

        // model nodes
        std::vector<ModelNodePtr> modelNodes = model->getModelNodes(modelNodeNames);
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!modelNodes.at(i)->ModelNodeType() != ModelNode::Link, "ModelNode "+ modelNodeNames[i] + " not of type Link.");
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
        ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);
        ModelNodePtr tcp = checkTCP(tcpName, model);

        LinkSetPtr mns = createLinkSet(model, name, modelNodes, kinematicRoot, tcp, registerToModel);
        return mns;
    }

    LinkSetPtr LinkSet::createLinkSet(const ModelPtr& model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const ModelNodePtr tcp, bool registerToModel)
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
            if (model)
                collisionChecker = model->getCollisionChecker();
            for (int i = 0; i < modelNodes.size(); i++)
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
        ModelNodePtr kinematicRootNode = checkKinematicRoot(kinematicRootName, model);
        ModelNodePtr tcpNode = checkTCP(tcpName, model);

        LinkSetPtr mns(new LinkSet(name, model, modelNodes, kinematicRootNode, tcpNode));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasLinkSet(mns), "LinkSet with name " + name + " already present in the model");
            model->registerLinkSet(mns);
        }

        return mns;
    }


    void LinkSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "LinkSet:" << endl;
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


    std::vector<CollisionModelPtr> LinkSet::getCollisionModels()
    {
        std::vector<CollisionModelPtr> collisionModels;

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            collisionModels.push_back(link->getCollisionModel());
        }

        return collisionModels;
    }

    CollisionCheckerPtr LinkSet::getCollisionChecker() const
    {
        if (getModel())
            return getModel()->getCollisionChecker();
        else
            return CollisionChecker::getGlobalCollisionChecker();
    }



    bool LinkSet::isKinematicChain()
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

    int LinkSet::getNumFaces(bool collisionModel)
    {
        int result = 0;

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            result += link->getNumFaces(collisionModel);
        }

        return result;
    }

    Eigen::Vector3f LinkSet::getCoM()
    {
        Eigen::Vector3f res;
        res.setZero();

        float m = getMass();

        if (m <= 0)
        {
            return res;
        }

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            res += link->getCoMGlobal() * link->getMass() / m;
        }

        return res;
    }

    float LinkSet::getMass()
    {
        float result = 0;

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            result += link->getMass();
        }

        return result;
    }


    std::string LinkSet::toXML(int tabs) const
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<LinkSet name='" << name; << "' kinematicRoot='" << kinematicRoot << "' tcp='" << tcp << "' >\n";

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ss << pre << t << "<Node name='" << modelNodes[i]->getName() << "'/>\n";
        }

        ss << pre << "</LinkSet>\n";
        return ss.str();
    }


    const std::vector<ModelLinkPtr> LinkSet::getLinks() const
    {
        std::vector<ModelLinkPtr> res;
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            res.push_back(link);
        }
        return res;
    }

}
