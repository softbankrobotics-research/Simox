
#include "ModelConfig.h"
#include "../VirtualRobotException.h"
#include "../Model/Nodes/ModelJoint.h"

namespace VirtualRobot
{

    ModelConfig::ModelConfig(ModelWeakPtr model, const std::string& name)
        : name(name),
          model(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "NULL model in ModelConfig");
    }

    ModelConfig::ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< Configuration >& configs)
        : name(name),
          model(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "NULL model in ModelConfig");

        for (std::vector< Configuration >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            setConfig((*i));
        }
    }

    ModelConfig::ModelConfig(ModelWeakPtr model, const std::string& name, const std::map< ModelJointPtr, float >& configs)
        : name(name),
          model(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "NULL model in ModelConfig");

        for (std::map< ModelJointPtr, float >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            setConfig(i->first, i->second);
        }
    }

    ModelConfig::ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< std::string >& modelNodes, const std::vector< float >& values)
        : name(name),
          model(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "NULL model in ModelConfig");
        THROW_VR_EXCEPTION_IF(modelNodes.size() != values.size(), "Vector sizes have to be equal in ModelConfig");

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            setConfig(modelNodes[i], values[i]);
        }
    }

    ModelConfig::ModelConfig(ModelWeakPtr model, const std::string& name, const std::vector< ModelJointPtr >& modelNodes, const std::vector< float >& values)
        : name(name),
          model(model)
    {
        THROW_VR_EXCEPTION_IF(!model.lock(), "NULL model in ModelConfig");
        THROW_VR_EXCEPTION_IF(modelNodes.size() != values.size(), "Vector sizes have to be equal in ModelConfig");

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            setConfig(modelNodes[i], values[i]);
        }
    }


    void ModelConfig::print() const
    {
        cout << "  Model Config <" << name << ">" << endl;

        for (std::map< ModelJointPtr, float >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            cout << "  * " << i->first->getName() << ":\t" << i->second << endl;
        }
    }

    bool ModelConfig::setConfig(const Configuration& c)
    {
        return setConfig(c.name, c.value);
    }

    bool ModelConfig::setConfig(const std::string& node, float value)
    {
        ModelPtr r = model.lock();

        if (!r)
        {
            VR_WARNING << "Model is already deleted, skipping update..." << endl;
            return false;
        }

        ModelJointPtr rn = r->getJoint(node);

        if (!rn)
        {
            VR_WARNING << "Did not find joint with name " << node << endl;
            return false;
        }

        configs[rn] = value;
        return true;
    }

    bool ModelConfig::setConfig(ModelJointPtr node, float value)
    {
        THROW_VR_EXCEPTION_IF(!node, "Null data");

        ModelPtr r = model.lock();

        if (!r)
        {
            VR_WARNING << "Model is already deleted, skipping update..." << endl;
            return false;
        }

        THROW_VR_EXCEPTION_IF(!r->hasJoint(node->getName()), "ModelNode with name " << r->getName() << " does not belong to model " << r->getName());

        configs[node] = value;
        return true;
    }

    VirtualRobot::ModelPtr ModelConfig::getModel()
    {
        return model.lock();
    }

    std::string ModelConfig::getName() const
    {
        return name;
    }

    ModelConfigPtr ModelConfig::clone(const ModelPtr &newModel)
    {
        ModelPtr model = newModel;
        if (!model)
        {
            model = this->model.lock();
        }

        VR_ASSERT(model);

        std::map< ModelJointPtr, float > newConfigs;
        std::map< ModelJointPtr, float >::iterator i = configs.begin();

        while (i != configs.end())
        {
            ModelJointPtr rn = model->getJoint(i->first->getName());

            if (!rn)
            {
                VR_WARNING << "Could not completely clone ModelConfig " << name << " because new model does not know a ModelNode with name " << i->first->getName() << endl;
            }
            else
            {
                newConfigs[rn] = i->second;
            }

            i++;
        }

        ModelConfigPtr result(new ModelConfig(model, name, newConfigs));
        return result;
    }


    bool ModelConfig::setJointValues()
    {
        ModelPtr r = model.lock();

        if (!r)
        {
            VR_WARNING << "Model is already deleted, skipping update..." << endl;
            return false;
        }

        WriteLockPtr lock = r->getWriteLock();

        for (std::map< ModelJointPtr, float >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            i->first->setJointValueNoUpdate(i->second);
        }

        r->applyJointValues();
        return true;
    }

    bool ModelConfig::hasConfig(const std::string& name) const
    {
        for (std::map< ModelJointPtr, float >::const_iterator i = configs.begin(); i != configs.end(); i++)
        {
            if (i->first->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    float ModelConfig::getConfig(const std::string& name) const
    {
        if (!hasConfig(name))
        {
            return 0.0f;
        }

        ModelPtr r = model.lock();

        if (!r)
        {
            VR_WARNING << "Model is already deleted..." << endl;
            return 0.0f;
        }

        ModelJointPtr rn = r->getJoint(name);
        THROW_VR_EXCEPTION_IF(!rn, "Did not find model node with name " << name);
        std::map< ModelJointPtr, float >::const_iterator i = configs.find(rn);

        if (i == configs.end())
        {
            VR_ERROR << "Internal error..." << endl;
            return 0.0f;
        }

        return i->second;
    }

    std::vector< ModelJointPtr > ModelConfig::getNodes() const
    {
        std::vector< ModelJointPtr > result;
        std::map< ModelJointPtr, float >::const_iterator i = configs.begin();

        while (i != configs.end())
        {
            result.push_back(i->first);
            i++;
        }

        return result;
    }

    std::map < std::string, float > ModelConfig::getJointNameValueMap()
    {
        std::map < std::string, float > result;
        std::map< ModelJointPtr, float >::const_iterator i = configs.begin();

        while (i != configs.end())
        {
            result[i->first->getName()] = i->second;
            i++;
        }

        return result;
    }

    bool ModelConfig::setJointValues(ModelPtr r)
    {
        if (!r)
        {
            return false;
        }

        WriteLockPtr lock = r->getWriteLock();

        std::map < std::string, float > jv = getJointNameValueMap();
        std::map< std::string, float >::const_iterator i = jv.begin();

        // first check if all nodes are present
        while (i != jv.end())
        {
            if (!r->hasModelNode(i->first))
            {
                return false;
            }

            i++;
        }

        // apply jv
        i = jv.begin();

        while (i != jv.end())
        {
            ModelJointPtr rn = r->getJoint(i->first);

            if (!rn)
            {
                return false;
            }

            rn->setJointValueNoUpdate(i->second);
            i++;
        }

        r->applyJointValues();
        return true;
    }

    std::string ModelConfig::toXML(int tabs)
    {
        std::map < std::string, float > jv = getJointNameValueMap();
        return createXMLString(jv, name, tabs);
    }

    std::string ModelConfig::createXMLString(const std::map< std::string, float >& config, const std::string& name, int tabs/*=0*/)
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        std::string tt = t;
        tt += "\t";
        std::string ttt = tt;
        ttt += "\t";

        ss << t << "<Configuration name='" << name << "'>\n";


        std::map< std::string, float >::const_iterator i = config.begin();

        while (i != config.end())
        {
            ss << tt << "<Node name='" << i->first << "' unit='radian' value='" << i->second << "'/>\n";
            i++;
        }

        ss << t << "</Configuration>\n";

        return ss.str();
    }

} // namespace VirtualRobot
