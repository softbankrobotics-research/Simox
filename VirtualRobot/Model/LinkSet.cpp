#include "LinkSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"
#include "../CollisionDetection/CollisionChecker.h"

namespace VirtualRobot
    {
    LinkSetPtr LinkSet::createLinkSet(const ModelPtr &model, const std::string &name, const std::vector<std::string> &linkNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        // model nodes
        std::vector<ModelLinkPtr> linkNodes;
        linkNodes.reserve(linkNames.size());
        for (const auto& nodeName : linkNames)
        {
            ModelLinkPtr node = model->getLink(nodeName);
            THROW_VR_EXCEPTION_IF(!node, "No ModelNode with name " + nodeName + " found.");
            linkNodes.push_back(node);
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
        ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);

        //tcp
        FramePtr tcp = checkTcp(tcpName, model);

        return createLinkSet(model, name, linkNodes, kinematicRoot, tcp, registerToModel);
    }

    LinkSetPtr LinkSet::createLinkSet(const ModelPtr &model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
    {
        std::vector<ModelLinkPtr> links;
        for (const auto & node : modelNodes)
        {
            ModelLinkPtr link = std::dynamic_pointer_cast<ModelLink>(node);
            if (link)
            {
                links.push_back(link);
            }
        }
        return createLinkSet(model, name, links, kinematicRoot, tcp, registerToModel);
    }

    LinkSetPtr LinkSet::createLinkSet(const ModelPtr &model, const std::string &name, const std::vector<ModelLinkPtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
    {
        LinkSetPtr mns(new LinkSet(name, model, modelNodes, kinematicRoot, tcp));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasNodeSet(mns), "NodeSet with name " + name + " already present in the model");
            model->registerNodeSet(mns);
        }

        return mns;
    }

    std::vector<ModelNodePtr> convertToNode(const std::vector<ModelLinkPtr> &linkNodes)
    {
        std::vector<ModelNodePtr> ret;
        ret.reserve(linkNodes.size());
        for (const auto& n : linkNodes)
        {
            ret.push_back(n);
        }
        return ret;
    }

    LinkSet::LinkSet(const std::string &name, const ModelWeakPtr &model, const std::vector<ModelLinkPtr> &linkNodes, const ModelNodePtr &kinematicRoot, const FramePtr &tcp) :
        ModelNodeSet(name, model, convertToNode(linkNodes), kinematicRoot, tcp),
        links(linkNodes),
        kinematicRoot(kinematicRoot),
        tcp(tcp)
    {
        if (!links.empty())
        {
            if (!tcp)
            {
                this->tcp = links.at(links.size()-1);
            }

            if (!kinematicRoot)
            {
                this->kinematicRoot = links.at(0);
            }
        }
    }

    LinkSet::~LinkSet()
    {

    }

    ModelNodePtr LinkSet::getNode(size_t i) const
    {
        return links.at(i);
    }

    ModelLinkPtr LinkSet::getLink(size_t i) const
    {
        return links.at(i);
    }

    bool LinkSet::hasNode(const ModelNodePtr &node) const
    {
        return std::find(links.begin(), links.end(), node) != links.end();
    }

    bool LinkSet::hasNode(const std::string &nodeName) const
    {
        for (const auto& node : getLinks())
        {
            if (node->getName() == nodeName)
            {
                return true;
            }
        }
        return false;
    }

    std::vector<ModelNodePtr> LinkSet::getNodes() const
    {
        return convertToNode(links);
    }

    std::vector<ModelJointPtr> LinkSet::getJoints() const
    {
        return std::vector<ModelJointPtr>();
    }

    std::vector<ModelLinkPtr> LinkSet::getLinks() const
    {
        return links;
    }

    unsigned int LinkSet::getSize() const
    {
        return links.size();
    }

    ModelNodePtr LinkSet::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void LinkSet::setKinematicRoot(const ModelNodePtr &modelNode)
    {
        THROW_VR_EXCEPTION_IF(!modelNode, "New kinematicRoot does not exist.");
        kinematicRoot = modelNode;
    }

    FramePtr LinkSet::getTCP() const
    {
        return tcp;
    }

    void LinkSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "LinkSet:" << endl;
        std::cout << "Name: " << getName() << endl;

        std::string modelName = getModel()->getName();

        std::cout << "Model Name: " << modelName << endl;
        std::cout << "Kinematic root: " << (getKinematicRoot() ? getKinematicRoot()->getName() : "") << endl;
        std::cout << "TCP: " << (getTCP() ? getTCP()->getName() : "") << endl;
        std::cout << "ModelNodes:" << endl;

        for (const auto& l : getLinks())
        {
            cout << "--ModelNode Name: " << l->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
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

        std::string krName = "";
        if (getKinematicRoot())
        {
            krName = getKinematicRoot()->getName();
        }
        std::string tcpName = "";
        if (getTCP())
        {
            tcpName = getTCP()->getName();
        }
        ss << pre << "<LinkSet name='" << getName() << "' kinematicRoot='" << krName << "' tcp='" << tcpName << "' >\n";

        for (const auto& l : getLinks())
        {
            ss << pre << t << "<Node name='" << l->getName() << "'/>\n";
        }

        ss << pre << "</LinkSet>\n";
        return ss.str();
    }

    ModelNodeSetPtr LinkSet::clone(const ModelPtr &model, const std::string &newName, bool registerToModel) const
    {
        std::vector<ModelLinkPtr> newModelLinks;
        for (const auto &n : getLinks())
        {
            THROW_VR_EXCEPTION_IF(!model->hasNode(n->getName()), "Cannot clone, new model does not contain node " << n->getName());
            ModelLinkPtr newModelLink = model->getLink(n->getName());
            THROW_VR_EXCEPTION_IF(!newModelLink, "The node \"" << n->getName() << "\" is not a link in the new model.");
            newModelLinks.push_back(newModelLink);
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

        LinkSetPtr result = LinkSet::createLinkSet(model, (newName.empty() ? getName() : newName), newModelLinks, newKinRoot, newTcp, registerToModel);

        return result;
    }

    std::vector<CollisionModelPtr> LinkSet::getCollisionModels() const
    {
        std::vector<CollisionModelPtr> collisionModels;

        for (const auto& n : getLinks())
        {
            collisionModels.push_back(n->getCollisionModel());
        }

        return collisionModels;
    }

    CollisionCheckerPtr LinkSet::getCollisionChecker() const
    {
        if (getModel())
        {
            return getModel()->getCollisionChecker();
        }
        else
        {
            return CollisionChecker::getGlobalCollisionChecker();
        }
    }

    int LinkSet::getNumFaces(bool collisionModel)
    {
        int result = 0;

        for (const auto& l : getLinks())
        {
            result += l->getNumFaces(collisionModel);
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

        for (const auto& l : getLinks())
        {
            res += l->getCoMGlobal() * l->getMass() / m;
        }

        return res;
    }

    float LinkSet::getMass()
    {
        float result = 0;

        for (const auto& l : getLinks())
        {
            result += l->getMass();
        }

        return result;
    }
}
