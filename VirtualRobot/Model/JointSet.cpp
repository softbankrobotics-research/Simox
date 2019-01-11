#include "JointSet.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"

namespace VirtualRobot
{
    JointSetPtr JointSet::createJointSet(const ModelPtr &model, const std::string &name, const std::vector<std::string> &jointNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
        THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");

        // model nodes
        std::vector<ModelJointPtr> jointNodes;
        jointNodes.reserve(jointNames.size());
        for (const auto& nodeName : jointNames)
        {
            ModelJointPtr node = model->getJoint(nodeName);
            THROW_VR_EXCEPTION_IF(!node, "No Joint with name " + nodeName + " found.");
            jointNodes.push_back(node);
        }

        // kinematic root
        // We do not check whether the given kinematic root is actually a root node.
        ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);

        //tcp
        FramePtr tcp = checkTcp(tcpName, model);

        return createJointSet(model, name, jointNodes, kinematicRoot, tcp, registerToModel);
    }

    JointSetPtr JointSet::createJointSet(const ModelPtr &model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
    {
        std::vector<ModelJointPtr> joints;
        for (const auto & node : modelNodes)
        {
            ModelJointPtr joint = std::dynamic_pointer_cast<ModelJoint>(node);
            if (joint)
            {
                joints.push_back(joint);
            }
        }
        return createJointSet(model, name, joints, kinematicRoot, tcp, registerToModel);
    }

    JointSetPtr JointSet::createJointSet(const ModelPtr &model, const std::string &name, const std::vector<ModelJointPtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
    {
        JointSetPtr mns(new JointSet(name, model, modelNodes, kinematicRoot, tcp));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasNodeSet(mns), "NodeSet with name " + name + " already present in the model");
            model->registerNodeSet(mns);
        }

        return mns;
    }

    std::vector<ModelNodePtr> convertToNode(const std::vector<ModelJointPtr> &jointNodes)
    {
        std::vector<ModelNodePtr> ret;
        ret.reserve(jointNodes.size());
        for (const auto& n : jointNodes)
        {
            ret.push_back(n);
        }
        return ret;
    }

    JointSet::JointSet(const std::string &name, const ModelWeakPtr &model, const std::vector<ModelJointPtr> &jointNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp) :
        ModelNodeSet(name, model, convertToNode(jointNodes), kinematicRoot, tcp),
        joints(jointNodes),
        kinematicRoot(kinematicRoot),
        tcp(tcp)
    {
        if (!joints.empty())
        {
            if (!tcp)
            {
                this->tcp = joints.at(joints.size()-1);
            }

            if (!kinematicRoot)
            {
                this->kinematicRoot = joints.at(0);
            }
        }
    }

    JointSet::~JointSet()
    {

    }

    ModelNodePtr JointSet::getNode(size_t i) const
    {
        return joints.at(i);
    }

    ModelJointPtr JointSet::getJoint(size_t i) const
    {
        return joints.at(i);
    }

    bool JointSet::hasNode(const ModelNodePtr &node) const
    {
        return std::find(joints.begin(), joints.end(), node) != joints.end();
    }

    bool JointSet::hasNode(const std::string &nodeName) const
    {
        for (const auto& node : getJoints())
        {
            if (node->getName() == nodeName)
            {
                return true;
            }
        }
        return false;
    }

    std::vector<ModelNodePtr> JointSet::getNodes() const
    {
        return convertToNode(joints);
    }

    std::vector<ModelJointPtr> JointSet::getJoints() const
    {
        return joints;
    }

    std::vector<ModelLinkPtr> JointSet::getLinks() const
    {
        return std::vector<ModelLinkPtr>();
    }

    unsigned int JointSet::getSize() const
    {
        return joints.size();
    }

    ModelNodePtr JointSet::getKinematicRoot() const
    {
        return kinematicRoot;
    }

    void JointSet::setKinematicRoot(const ModelNodePtr &modelNode)
    {
        THROW_VR_EXCEPTION_IF(!modelNode, "New kinematicRoot does not exist.");
        kinematicRoot = modelNode;
    }

    FramePtr JointSet::getTCP() const
    {
        return tcp;
    }

    void JointSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "JointSet:" << endl;
        std::cout << "Name: " << getName() << endl;

        std::string modelName = getModel()->getName();

        std::cout << "Model Name: " << modelName << endl;
        std::cout << "Kinematic root: " << (getKinematicRoot() ? getKinematicRoot()->getName() : "") << endl;
        std::cout << "TCP: " << (getTCP() ? getTCP()->getName() : "") << endl;
        std::cout << "ModelNodes:" << endl;

        for (const auto& node : getJoints())
        {
            cout << "--ModelNode Name: " << node->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
    }

    std::string JointSet::toXML(int tabs) const
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
        ss << pre << "<JointSet name='" << getName() << "' kinematicRoot='" << krName << "' tcp='" << tcpName << "' >\n";

        for (const auto& n : getJoints())
        {
            ss << pre << t << "<Node name='" << n->getName() << "'/>\n";
        }

        ss << pre << "</JointSet>\n";
        return ss.str();
    }

    ModelNodeSetPtr JointSet::clone(const ModelPtr &model, const std::string &newName, bool registerToModel) const
    {
        std::vector<ModelJointPtr> newModelJoints;
        for (const auto &n : getJoints())
        {
            THROW_VR_EXCEPTION_IF(!model->hasNode(n->getName()), "Cannot clone, new model does not contain node " << n->getName());
            ModelJointPtr newModelJoint = model->getJoint(n->getName());
            THROW_VR_EXCEPTION_IF(!newModelJoint, "The node \"" << n->getName() << "\" is not a link in the new model.");
            newModelJoints.push_back(newModelJoint);
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

        JointSetPtr result = JointSet::createJointSet(model, (newName.empty() ? getName() : newName), newModelJoints, newKinRoot, newTcp, registerToModel);

        return result;
    }

    std::vector<float> JointSet::getJointValues() const
    {
        std::vector<float> fillVector;
        getJointValues(fillVector);

        return fillVector;
    }

    void JointSet::getJointValues(std::vector<float> &fillVector, bool clearVector) const
    {
        if (clearVector)
        {
            fillVector.clear();
        }

        std::vector<ModelJointPtr> modelJoints = getJoints();

        for (const ModelJointPtr & joint : modelJoints)
        {
            fillVector.push_back(joint->getJointValue());
        }
    }

    void JointSet::getJointValues(Eigen::VectorXf &fillVector) const
    {
        std::vector<float> jointValues = getJointValues();
        fillVector.resize(jointValues.size());

        for (size_t i = 0; i < jointValues.size(); i++)
        {
            fillVector[i] = jointValues[i];
        }
    }

    void JointSet::getJointValues(const ModelConfigPtr& config) const
    {
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        for (size_t i = 0; i < joints.size(); i++)
        {
            config->setConfig(joints[i]->getName(), joints[i]->getJointValue());
        }
    }

    void JointSet::respectJointLimits(std::vector<float>& jointValues) const
    {
        std::vector<ModelJointPtr> modelJoints = getJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    void JointSet::respectJointLimits(Eigen::VectorXf& jointValues) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        std::vector<ModelJointPtr> modelJoints = getJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    bool JointSet::checkJointLimits(const std::vector<float>& jointValues, bool verbose) const
    {
        std::vector<ModelJointPtr> modelJoints = getJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            if (!modelJoints[i]->checkJointLimits(jointValues[i], verbose))
            {
                return false;
            }
        }
        return true;
    }

    bool JointSet::checkJointLimits(const Eigen::VectorXf& jointValues, bool verbose) const
    {
        std::vector<ModelJointPtr> modelJoints = getJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            if (!modelJoints[i]->checkJointLimits(jointValues[i], verbose))
            {
                return false;
            }
        }
        return true;
    }

    void JointSet::setJointValues(const std::vector<float>& jointValues)
    {
        std::vector<ModelJointPtr> modelJoints = getJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        ModelPtr model = weakModel.lock();
        VR_ASSERT(model);
        WriteLockPtr lock = model->getWriteLock();

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

    void JointSet::setJointValues(const Eigen::VectorXf& jointValues)
    {
        std::vector<ModelJointPtr> modelJoints = getJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        ModelPtr model = weakModel.lock();
        VR_ASSERT(model);
        WriteLockPtr lock = model->getWriteLock();

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

    void JointSet::setJointValues(const ModelConfigPtr& config)
    {
        config->setJointValues(getModel());
    }
    
    std::map<std::string, float> JointSet::getJointValueMap() const
    {
        std::map<std::string, float> res;
        for (auto n: joints)
        {
            res[n->getName()] = n->getJointValue();
        }
        return res;
    }

    bool JointSet::isJointSet() const
    {
        return true;
    }
}
