#include "JointSet.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "ModelConfig.h"

namespace VirtualRobot
{

    JointSet::JointSet(const std::string &name, const ModelWeakPtr& model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const FramePtr tcp) :
        ModelNodeSet(name, model, modelNodes, kinematicRoot, tcp)
    {
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ModelJointPtr j = std::static_pointer_cast<ModelJoint>(modelNodes[i]);
            THROW_VR_EXCEPTION_IF(!j, "Only joints allowed in joint sets.");
            joints.push_back(j);
        }
    }

	JointSet::~JointSet()
	{
	}

    JointSetPtr JointSet::createJointSet(const ModelPtr& model, const std::string &name, const std::vector<std::string> &modelNodeNames, const std::string &kinematicRootName, const std::string &tcpName, bool registerToModel)
    {
		THROW_VR_EXCEPTION_IF(!model, "Model not initialized.");
		THROW_VR_EXCEPTION_IF(modelNodeNames.empty(), "Empty set of ModelNode names provided.");

        // model nodes
        std::vector<ModelNodePtr> modelNodes = model->getModelNodes(modelNodeNames);
        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!(modelNodes.at(i)->getType() & ModelNode::Joint), "ModelNode "+ modelNodeNames[i] + " not of type Joint.");
        }

        ModelNodePtr kinematicRoot = checkKinematicRoot(kinematicRootName, model);
        ModelNodePtr tcp = checkTcp(tcpName, model);

        JointSetPtr mns = createJointSet(model, name, modelNodes, kinematicRoot, tcp, registerToModel);
        return mns;
    }
	
	ModelNodeSetPtr JointSet::clone(ModelPtr newModel)
	{
		// todo
		return JointSetPtr();
	}

    JointSetPtr JointSet::createJointSet(const ModelPtr& model, const std::string &name, const std::vector<ModelNodePtr> &modelNodes, const ModelNodePtr kinematicRoot, const FramePtr tcp, bool registerToModel)
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
        }

        JointSetPtr mns(new JointSet(name, model, modelNodes, kinematicRoot, tcp));

        if (registerToModel)
        {
            THROW_VR_EXCEPTION_IF(model->hasJointSet(mns), "JointSet with name " + name + " already present in the model");
            model->registerJointSet(mns);
        }

        return mns;
    }

    void JointSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "JointSet:" << endl;
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

	ModelJointPtr &JointSet::getNode(int i)
	{
		THROW_VR_EXCEPTION_IF((i >= (int)joints.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (joints.size() - 1));
		return joints.at(i);
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

        std::vector<ModelJointPtr> modelJoints = getModelJoints();

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
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    void JointSet::respectJointLimits(Eigen::VectorXf& jointValues) const
    {
        THROW_VR_EXCEPTION_IF(jointValues.size() != getModelJoints().size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        for (size_t i = 0; i < modelJoints.size(); i++)
        {
            modelJoints[i]->respectJointLimits(jointValues[i]);
        }
    }

    bool JointSet::checkJointLimits(const std::vector<float>& jointValues, bool verbose) const
    {
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

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
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

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
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

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
        std::vector<ModelJointPtr> modelJoints = getModelJoints();
        THROW_VR_EXCEPTION_IF(jointValues.size() != modelJoints.size(),
                              "Wrong vector dimension (modelNodes:" << getModelJoints().size() << ", jointValues: " << jointValues.size() << ")" << endl);

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

    std::string JointSet::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<JointSet name='" << name << "' kinematicRoot='" << kinematicRoot << "' tcp='" << tcp << "' >\n";

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            ss << pre << t << "<Node name='" << modelNodes[i]->getName() << "'/>\n";
        }

        ss << pre << "</JointSet>\n";
        return ss.str();
    }

    std::vector<std::string> JointSet::getNodeNames() const
    {
        std::vector<std::string> res;
        for (auto n: joints)
            res.push_back(n->getName());
        return res;
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


    const std::vector<ModelJointPtr> JointSet::getJoints() const
    {
        return joints;
    }

}
