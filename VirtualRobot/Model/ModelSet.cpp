#include "ModelSet.h"
#include "Nodes/ModelLink.h"
#include "Nodes/ModelJoint.h"
#include "../VirtualRobotException.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../Visualization/VisualizationSet.h"
#include "../Visualization/VisualizationFactory.h"
#include "../Model/Obstacle.h"
#include "../CollisionDetection/CollisionModel.h"
#include "ModelConfig.h"

namespace VirtualRobot
{

    ModelSet::ModelSet(const std::string &name,
        const std::vector<ModelPtr> &models):
            name(name),
            models(models)
    {
        for (size_t i = 0; i< models.size(); i++)
        {
            for (size_t j = i+1; j < models.size(); j++)
            {
                VR_ASSERT(models[i]->getCollisionChecker() == models[j]->getCollisionChecker());
            }
        }
    }

    ModelSet::~ModelSet()
	{

	}

    ModelSetPtr ModelSet::clone(const std::string &name)
	{
        ModelSetPtr result(new ModelSet(name, models));
        return result;
    }

    ObstaclePtr ModelSet::createStaticObstacle(const std::string &name) const
    {
        auto visus = getVisualizations();
        auto colModels = getCollisionModels();
        VisualizationPtr visu = VisualizationFactory::getGlobalVisualizationFactory()->createUnitedVisualization(visus);
        CollisionModelPtr colModel = CollisionModel::CreateUnitedCollisionModel(colModels);
        ObstaclePtr result = Obstacle::create(name, visu, colModel);
        return result;
    }


    std::string ModelSet::getName() const
    {
        return name;
    }

    ModelPtr &ModelSet::getModel(int i)
    {
        THROW_VR_EXCEPTION_IF((i >= (int)models.size() || i < 0), "Index out of bounds:" << i << ", (should be between 0 and " << (models.size() - 1));
        return models.at(i);
    }

    std::vector<ModelPtr>::iterator ModelSet::begin()
    {
        return models.begin();
    }

    std::vector<ModelPtr>::iterator ModelSet::end()
    {
        return models.end();
    }

    bool ModelSet::hasModel(const ModelPtr & model) const
    {
        return std::find(models.begin(), models.end(), model) != models.end();
    }

    bool ModelSet::hasModel(const std::string &modelName) const
    {
        for (const ModelPtr & node : models)
        {
            if (node->getName() == modelName)
            {
                return true;
            }
        }
        return false;
    }

    const std::vector<ModelPtr> ModelSet::getModels() const
    {
        return models;
    }

    void ModelSet::print() const
    {
        std::cout << "----------------------------------------------" << endl;
        std::cout << "ModelSet:" << endl;
        std::cout << "Name: " << name << endl;
        std::cout << "Models:" << endl;

        for (const ModelPtr & node : models)
        {
            cout << "--Model Name: " << node->getName() << endl;
        }
        std::cout << "----------------------------------------------" << endl;
    }

    unsigned int ModelSet::getSize() const
    {
        return models.size();
    }

    std::string ModelSet::toXML(int tabs) const
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<ModelSet name='" << name << "'>\n";

        for (size_t i = 0; i < models.size(); i++)
        {
            ss << pre << t << "<Model name='" << models[i]->getName() << "'/>\n";
        }

        ss << pre << "</ModelSet>\n";
        return ss.str();
    }
    
    std::vector<ModelJointPtr> ModelSet::getModelJoints() const
    {
        std::vector<ModelJointPtr> modelJoints;
        for (const ModelPtr &node : models)
        {
            std::vector<ModelJointPtr> jm = node->getJoints();
            for (auto &j : jm)
            {
                modelJoints.push_back(j);
            }
        }
        return modelJoints;
    }

    std::vector<ModelLinkPtr> ModelSet::getModelLinks() const
    {
        std::vector<ModelLinkPtr> modelLinks;
        for (const ModelPtr &node : models)
        {
            std::vector<ModelLinkPtr> jm = node->getLinks();
            for (auto &j : jm)
            {
                modelLinks.push_back(j);
            }
        }
        return modelLinks;
    }

    CollisionCheckerPtr ModelSet::getCollisionChecker() const
    {
        if (models.size()>0)
            return models.at(0)->getCollisionChecker();
        return CollisionChecker::getGlobalCollisionChecker();
    }
    
    std::vector<std::string> ModelSet::getModelNames() const
    {
        std::vector<std::string> res;
        for (auto n: models)
            res.push_back(n->getName());
        return res;
    }

    std::vector<CollisionModelPtr> ModelSet::getCollisionModels() const
    {
        std::vector<CollisionModelPtr> res;
        for (auto &m : models)
        {
            std::vector<CollisionModelPtr> cm = m->getCollisionModels();
            if (cm.size()>0)
                res.insert(res.end(), cm.begin(), cm.end());
        }
        return res;
    }

    std::vector<VisualizationPtr> ModelSet::getVisualizations() const
    {
        std::vector<VisualizationPtr> res;
        for (const ModelPtr &m : models)
        {
            auto visu = m->getVisualization(ModelLink::Full);
            if (!visu)
                continue;
            res.push_back(visu);
        }
        return res;
    }
}
