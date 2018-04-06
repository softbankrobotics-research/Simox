#include "LinkSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"
#include "../CollisionDetection/CollisionChecker.h"

namespace VirtualRobot
{

    LinkSet::LinkSet(const std::string &name, const ModelWeakPtr& model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const FramePtr tcp) :
            ModelNodeSet(name, model, modelNodes, kinematicRoot, tcp)
    {
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelLinkPtr j = std::static_pointer_cast<ModelLink>(modelNodes[i]);
            THROW_VR_EXCEPTION_IF(!j, "Only links allowed in link sets.");
            links.push_back(j);
        }
    }

	LinkSet::~LinkSet()
	{
	}

    LinkSetPtr LinkSet::createLinkSet(
		const ModelPtr& model, 
		const std::string &name,
		const std::vector<std::string> &modelNodeNames, 
		const std::string &kinematicRootName, 
		const std::string &tcpName, 
		bool registerToModel)
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
            THROW_VR_EXCEPTION_IF(modelNodes.at(i)->getType() != ModelNode::Link, "ModelNode "+ modelNodeNames[i] + " not of type Link.");
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
		ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);
        ModelNodePtr tcp = checkTcp(tcpName, model);

        LinkSetPtr mns = createLinkSet(model, name, modelNodes, kinematicRoot, tcp, registerToModel);
        return mns;
    }

	ModelLinkPtr &LinkSet::getNode(int i)
	{
		return getLink(i);
	}

	ModelLinkPtr &LinkSet::getLink(int i)
	{
		THROW_VR_EXCEPTION_IF((i >= (int)links.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (links.size() - 1));
		return links.at(i);
	}

    LinkSetPtr LinkSet::createLinkSet(const ModelPtr& model, const std::string &name, const std::vector<ModelLinkPtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
    {
        std::vector<ModelNodePtr> links;
        for (unsigned int i = 0; i < modelNodes.size(); i++)
        {
            ModelNodePtr link = std::static_pointer_cast<ModelNode>(modelNodes[i]);
            links.push_back(link);
        }
        return createLinkSet(model, name, links, kinematicRoot, tcp, registerToModel);
    }

    LinkSetPtr LinkSet::createLinkSet(const ModelPtr& model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
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


        LinkSetPtr mns(new LinkSet(name, model, modelNodes, kinematicRoot, tcp));

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
            ModelLinkPtr link = std::static_pointer_cast<ModelLink>(modelNodes.at(i));
            VR_ASSERT(link);
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

        ss << pre << "<LinkSet name='" << name << "' kinematicRoot='" << kinematicRoot << "' tcp='" << tcp << "' >\n";

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

    LinkSetPtr VirtualRobot::LinkSet::clone(ModelPtr model)
	{
        std::vector<ModelLinkPtr> newModelNodes;
        for (auto &n: links)
        {
            THROW_VR_EXCEPTION_IF(!model->hasLink(n->getName()), "Cannot clone, new model does not contain link " << n->getName());
            ModelLinkPtr no = model->getLink(n->getName());
            VR_ASSERT(no);
            newModelNodes.push_back(no);
        }
        ModelNodePtr newKinRoot;
        if (kinematicRoot)
        {
            newKinRoot = model->getModelNode(kinematicRoot->getName());
            VR_ASSERT(newKinRoot);
        }
        ModelNodePtr newTcp;
        if (tcp)
        {
            newTcp = model->getModelNode(tcp->getName());
            VR_ASSERT(newTcp);
        }

        LinkSetPtr result = LinkSet::createLinkSet(model, name, newModelNodes, newKinRoot, newTcp, true);
        return result;
	}
}
