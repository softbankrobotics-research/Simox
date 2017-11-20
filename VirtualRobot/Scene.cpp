
#include "Scene.h"
#include "VirtualRobotException.h"
#include "Model/ManipulationObject.h"
#include "Model/LinkSet.h"
#include "Trajectory.h"
#include "XML/BaseIO.h"
#include <VirtualRobot/Visualization/VisualizationSet.h>

namespace VirtualRobot
{

    Scene::Scene(const std::string& name)
        : name(name)
    {
    }

    Scene::~Scene()
    {
        robots.clear();
        obstacles.clear();
        manipulationObjects.clear();
        trajectories.clear();
    }

    void Scene::registerRobot(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (hasRobot(robot))
        {
            return;
        }

        robots.push_back(robot);
    }

    void Scene::deRegisterRobot(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (!hasRobot(robot))
        {
            return;
        }

        for (std::vector< RobotPtr >::iterator i = robots.begin(); i != robots.end(); i++)
        {
            if (*i == robot)
            {
                robots.erase(i);
                break;
            }
        }
    }

    void Scene::deRegisterRobot(const std::string& name)
    {
        if (!hasRobot(name))
        {
            return;
        }

        for (std::vector< RobotPtr >::iterator i = robots.begin(); i != robots.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                robots.erase(i);
                break;
            }
        }
    }

    void Scene::registerObstacle(ObstaclePtr obstacle)
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        if (hasObstacle(obstacle))
        {
            return;
        }

        obstacles.push_back(obstacle);
    }

    void Scene::deRegisterObstacle(ObstaclePtr obstacle)
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        if (!hasObstacle(obstacle))
        {
            return;
        }

        for (std::vector< ObstaclePtr >::iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i) == obstacle)
            {
                obstacles.erase(i);
                break;
            }
        }
    }

    void Scene::deRegisterObstacle(const std::string& name)
    {
        if (!hasObstacle(name))
        {
            return;
        }

        for (std::vector< ObstaclePtr >::iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                obstacles.erase(i);
                break;
            }
        }
    }



    void Scene::registerTrajectory(TrajectoryPtr t)
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        if (hasTrajectory(t))
        {
            return;
        }

        trajectories.push_back(t);
    }

    void Scene::deRegisterTrajectory(TrajectoryPtr t)
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        if (!hasTrajectory(t))
        {
            return;
        }

        for (std::vector< TrajectoryPtr >::iterator i = trajectories.begin(); i != trajectories.end(); i++)
        {
            if ((*i) == t)
            {
                trajectories.erase(i);
                break;
            }
        }
    }

    void Scene::deRegisterTrajectory(const std::string& name)
    {
        if (!hasTrajectory(name))
        {
            return;
        }

        for (std::vector< TrajectoryPtr >::iterator i = trajectories.begin(); i != trajectories.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                trajectories.erase(i);
                break;
            }
        }
    }

    void Scene::registerManipulationObject(ManipulationObjectPtr manipulationObject)
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        if (hasManipulationObject(manipulationObject))
        {
            return;
        }

        manipulationObjects.push_back(manipulationObject);
    }

    void Scene::deRegisterManipulationObject(ManipulationObjectPtr manipulationObject)
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        if (!hasManipulationObject(manipulationObject))
        {
            return;
        }

        for (std::vector< ManipulationObjectPtr >::iterator i = manipulationObjects.begin(); i != manipulationObjects.end(); i++)
        {
            if ((*i) == manipulationObject)
            {
                manipulationObjects.erase(i);
                break;
            }
        }
    }

    void Scene::deRegisterManipulationObject(const std::string& name)
    {
        if (!hasManipulationObject(name))
        {
            return;
        }

        for (std::vector< ManipulationObjectPtr >::iterator i = manipulationObjects.begin(); i != manipulationObjects.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                manipulationObjects.erase(i);
                break;
            }
        }
    }

    bool Scene::hasRobot(RobotPtr robot) const
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        for (std::vector< RobotPtr >::const_iterator i = robots.begin(); i != robots.end(); i++)
        {
            if (*i == robot)
            {
                return true;
            }
        }

        return false;
    }

    bool Scene::hasRobot(const std::string& name) const
    {
        for (std::vector< RobotPtr >::const_iterator i = robots.begin(); i != robots.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    bool Scene::hasObstacle(ObstaclePtr obstacle) const
    {
        THROW_VR_EXCEPTION_IF(!obstacle, "NULL data");

        for (std::vector< ObstaclePtr >::const_iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if (*i == obstacle)
            {
                return true;
            }
        }

        return false;
    }

    bool Scene::hasObstacle(const std::string& name) const
    {
        for (std::vector< ObstaclePtr >::const_iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return true;
            }
        }

        return false;
    }



    bool Scene::hasTrajectory(TrajectoryPtr t) const
    {
        THROW_VR_EXCEPTION_IF(!t, "NULL data");

        for (std::vector< TrajectoryPtr >::const_iterator i = trajectories.begin(); i != trajectories.end(); i++)
        {
            if (*i == t)
            {
                return true;
            }
        }

        return false;
    }

    bool Scene::hasTrajectory(const std::string& name) const
    {
        for (std::vector< TrajectoryPtr >::const_iterator i = trajectories.begin(); i != trajectories.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return true;
            }
        }

        return false;
    }


    bool Scene::hasManipulationObject(ManipulationObjectPtr manipulationObject) const
    {
        THROW_VR_EXCEPTION_IF(!manipulationObject, "NULL data");

        for (std::vector< ManipulationObjectPtr >::const_iterator i = manipulationObjects.begin(); i != manipulationObjects.end(); i++)
        {
            if (*i == manipulationObject)
            {
                return true;
            }
        }

        return false;
    }

    bool Scene::hasManipulationObject(const std::string& name) const
    {
        for (std::vector< ManipulationObjectPtr >::const_iterator i = manipulationObjects.begin(); i != manipulationObjects.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return true;
            }
        }

        return false;
    }

    VirtualRobot::RobotPtr Scene::getRobot(const std::string& name)
    {
        for (std::vector< RobotPtr >::const_iterator i = robots.begin(); i != robots.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No robot with name " << name << " registered in scene " << this->name << endl;
        return RobotPtr();
    }

    VirtualRobot::ObstaclePtr Scene::getObstacle(const std::string& name)
    {
        for (std::vector< ObstaclePtr >::const_iterator i = obstacles.begin(); i != obstacles.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No obstacle with name " << name << " registered in scene " << this->name << endl;
        return ObstaclePtr();
    }

    VirtualRobot::TrajectoryPtr Scene::getTrajectory(const std::string& name)
    {
        for (std::vector< TrajectoryPtr >::const_iterator i = trajectories.begin(); i != trajectories.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No Trajectory with name " << name << " registered in scene " << this->name << endl;
        return TrajectoryPtr();
    }

    VirtualRobot::ManipulationObjectPtr Scene::getManipulationObject(const std::string& name)
    {
        for (std::vector< ManipulationObjectPtr >::const_iterator i = manipulationObjects.begin(); i != manipulationObjects.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                return *i;
            }
        }

        VR_WARNING << "No ManipulationObject with name " << name << " registered in scene " << this->name << endl;
        return ManipulationObjectPtr();
    }

    void Scene::registerRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot data");
        THROW_VR_EXCEPTION_IF(!config, "NULL config data");

        robot->registerConfiguration(config);
    }

    void Scene::registerRobotConfig(RobotPtr robot, std::vector<RobotConfigPtr> configs)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL robot data");
        THROW_VR_EXCEPTION_IF(configs.size()==0, "No config data");

        robot->registerConfiguration(configs);
    }

    void Scene::deRegisterRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        robot->deRegisterConfiguration(config);
    }

    void Scene::deRegisterRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        robot->deRegisterConfiguration(name);
    }

    bool Scene::hasRobotConfig(RobotPtr robot, RobotConfigPtr config)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!config, "NULL data");

        return robot->hasConfiguration(config);
    }

    bool Scene::hasRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        return robot->hasConfiguration(name);
    }

    VirtualRobot::RobotConfigPtr Scene::getRobotConfig(const std::string& robotName, const std::string& name)
    {
        ModelPtr r = getRobot(robotName);
        if (!r)
        {
            VR_ERROR << "No model with name " << robotName << endl;
            return RobotConfigPtr();
        }
        return r->getConfiguration(name);
    }

    VirtualRobot::RobotConfigPtr Scene::getRobotConfig(RobotPtr robot, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        return robot->getConfiguration(name);
    }

    std::string Scene::getName() const
    {
        return name;
    }

    VisualizationGroupPtr Scene::getAllVisualizations(ModelLink::VisualizationType visuType , bool addRobots, bool addObstacles, bool addManipulationObjects, bool addTrajectories, bool addSceneObjectSets, bool addAttachments) const
    {
        std::vector<VisualizationPtr> collectedVisualizationNodes;

        if (addRobots)
        {
            std::vector<VirtualRobot::RobotPtr> robots = getRobots();
            for (auto& robot : robots)
            {
                auto visu = robot->getVisualization(visuType, addAttachments);
                collectedVisualizationNodes.push_back(visu);
            }
        }

        if (addObstacles)
        {
            std::vector<VirtualRobot::ObstaclePtr> obstacles = getObstacles();
            for (auto& obstacle : obstacles)
            {
                auto visu = obstacle->getVisualization(visuType, addAttachments);
                collectedVisualizationNodes.push_back(visu);
            }
        }

        if (addManipulationObjects)
        {
            std::vector<VirtualRobot::ManipulationObjectPtr> manipulationObjects = getManipulationObjects();
            for (auto& manipulationObject : manipulationObjects)
            {
                auto visu = manipulationObject->getVisualization(visuType, addAttachments);
                collectedVisualizationNodes.push_back(visu);
            }
        }

        if (addTrajectories)
        {
            std::vector<VirtualRobot::TrajectoryPtr> trajectories = getTrajectories();
            for (auto& trajectory : trajectories)
            {
                auto visu = trajectory->getVisualization();
                collectedVisualizationNodes.push_back(visu);
            }
        }

        if (addSceneObjectSets)
        {
            VR_WARNING << "nyi" << endl;
            //todo
        }

        return VisualizationGroupPtr(new VisualizationGroup(collectedVisualizationNodes));
    }

    std::vector< RobotPtr > Scene::getRobots() const
    {
        return robots;
    }

    std::vector< ObstaclePtr > Scene::getObstacles() const
    {
        return obstacles;
    }

    std::vector< TrajectoryPtr > Scene::getTrajectories() const
    {
        return trajectories;
    }

    std::vector< TrajectoryPtr > Scene::getTrajectories(const std::string& robotName) const
    {
        std::vector< TrajectoryPtr > res;

        for (size_t i = 0; i < trajectories.size(); i++)
        {
            if (trajectories[i]->getRobotName() == robotName)
            {
                res.push_back(trajectories[i]);
            }
        }

        return res;
    }

    void Scene::registerModelSet(const ModelSetPtr modelSet)
    {
        modelSets.push_back(modelSet);
    }

    void Scene::deRegisterModelSet(const std::string &name)
    {
        if (!hasModelSet(name))
        {
            return;
        }

        for (auto i = modelSets.begin(); i != modelSets.end(); i++)
        {
            if ((*i)->getName() == name)
            {
                modelSets.erase(i);
                break;
            }
        }

    }

    bool Scene::hasModelSet(const std::string &name)
    {
        for (auto &m : modelSets)
        {
            if (m->getName() == name)
                return true;
        }
        return false;
    }

    std::vector< ManipulationObjectPtr > Scene::getManipulationObjects() const
    {
        return manipulationObjects;
    }

    std::vector< RobotConfigPtr > Scene::getRobotConfigs(RobotPtr robot)
    {
        if (!robot || !hasRobot(robot->getName()))
        {
            return std::vector< RobotConfigPtr >();
        }

        return robot->getConfigurations();
    }
	
    VirtualRobot::ModelNodeSetPtr Scene::getModelNodeSet(const std::string& robot, const std::string rns)
    {
        RobotPtr r = getRobot(robot);

        if (!r)
        {
            VR_ERROR << " no robot with name " << robot << endl;
            return RobotNodeSetPtr();
        }

        return r->getModelNodeSet(rns);
    }



    std::vector<ModelSetPtr> Scene::getModelSets()
    {
        return modelSets;
    }

    ModelSetPtr Scene::getModelSet(const std::string &name)
    {
        if (!hasModelSet(name))
        {
            VR_WARNING << "No model set with name " << name << " registered" << endl;
            return ModelSetPtr();
        }
        for (auto &n: modelSets)
        {
            if (n->getName() == name)
                return n;
        }
        return ModelSetPtr();
    }


    std::string Scene::getXMLString(const std::string& basePath)
    {
        std::stringstream ss;
        ss << "<Scene";

        if (!name.empty())
        {
            ss << " name='" << name << "'";
        }

        ss << ">\n";
        ss << "\n";

        // process robots
        for (size_t i = 0; i < robots.size(); i++)
        {
            std::string rob = robots[i]->getName();
            RobotConfigPtr currentConfig = robots[i]->getConfig();
            ss << "\t<Robot name='" << rob << "' initConfig='" << currentConfig->getName() << "'>\n";
            std::string robFile = robots[i]->getFilename();

            if (!basePath.empty())
            {
                BaseIO::makeRelativePath(basePath, robFile);
            }

            ss << "\t\t<File>" << robFile << "</File>\n";

            // store global pose (if not identity)
            Eigen::Matrix4f gp = robots[i]->getGlobalPose();

            if (!gp.isIdentity())
            {
                ss << "\t\t<GlobalPose>\n";
                ss << "\t\t\t<Transform>\n";
                ss << BaseIO::getTransformXMLString(gp, 4);
                ss << "\t\t\t</Transform>\n";
                ss << "\t\t</GlobalPose>\n";
            }

            // store current config
            ss << currentConfig->toXML(2);

            // store all other configs for robot -> already included in robot
            /*std::vector<RobotConfigPtr> rc = getRobotConfigs(robots[i]);

            for (size_t j = 0; j < rc.size(); j++)
            {
                ss << rc[j]->toXML(2);
            }*/

            ss << "\t</Robot>\n";
            ss << "\n";
        }

        // process manipulation objects
        for (size_t i = 0; i < manipulationObjects.size(); i++)
        {
            ss << manipulationObjects[i]->toXML(basePath, 1, true);
            ss << "\n";
        }

        // process obstacles
        for (size_t i = 0; i < obstacles.size(); i++)
        {
            ss << obstacles[i]->toXML(basePath, 1);
            ss << "\n";
        }

        // process model sets
        for (size_t i = 0; i < modelSets.size(); i++)
        {
            ss << modelSets[i]->toXML(1);
            ss << "\n";
        }

        // process trajectories
        for (size_t i = 0; i < trajectories.size(); i++)
        {
            ss << trajectories[i]->toXML(1);
            ss << "\n";
        }

        ss << "</Scene>\n";

        return ss.str();
    }


} //  namespace


