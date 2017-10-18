/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Scene_h_
#define _VirtualRobot_Scene_h_

#include "Model/Model.h"
#include "Model/Nodes/ModelLink.h"
#include "Model/Model.h"
#include "Model/ModelConfig.h"
#include "Model/Nodes/ModelNode.h"
#include "Model/Obstacle.h"
#include "Trajectory.h"
#include "Model/ManipulationObject.h"
#include "Model/ModelSet.h"

#include <string>
#include <vector>
#include <Eigen/Core>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Scene  : public std::enable_shared_from_this<Scene>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
        */
        Scene(const std::string& name);

        /*!
        */
        virtual ~Scene();

        /*!
            Registers the robot to this scene. If a robot with the same name is already registered nothing happens.
        */
        void registerRobot(RobotPtr robot);

        /*!
            Removes the robot to from this scene. If the robot is not registered nothing happens.
        */
        void deRegisterRobot(RobotPtr robot);
        void deRegisterRobot(const std::string& name);

        bool hasRobot(RobotPtr robot) const;
        bool hasRobot(const std::string& name) const;

        RobotPtr getRobot(const std::string& name);

        std::vector< RobotPtr > getRobots();

        /*!
            Registers the RobotConfig to this scene. If a config  with the same name is already registered nothing happens.
        */
        void registerRobotConfig(RobotPtr robot, RobotConfigPtr config);
        void registerRobotConfig(RobotPtr robot, std::vector<RobotConfigPtr> configs);

        /*!
            Removes the RobotConfig to from this scene. If the RobotConfig is not registered nothing happens.
        */
        void deRegisterRobotConfig(RobotPtr robot, RobotConfigPtr config);
        void deRegisterRobotConfig(RobotPtr robot, const std::string& name);

        bool hasRobotConfig(RobotPtr robot, RobotConfigPtr config);
        bool hasRobotConfig(RobotPtr robot, const std::string& name);

        RobotConfigPtr getRobotConfig(const std::string& robotName, const std::string& name);
        RobotConfigPtr getRobotConfig(RobotPtr robot, const std::string& name);

        std::vector< RobotConfigPtr > getRobotConfigs(RobotPtr robot);

        /*!
            Registers the ManipulationObject to this scene. If an ManipulationObject with the same name is already registered nothing happens.
        */
        void registerManipulationObject(ManipulationObjectPtr obj);

        /*!
            Removes the ManipulationObject to from this scene. If the ManipulationObject is not registered nothing happens.
        */
        void deRegisterManipulationObject(ManipulationObjectPtr obj);
        void deRegisterManipulationObject(const std::string& name);

        bool hasManipulationObject(ManipulationObjectPtr obstacle) const;
        bool hasManipulationObject(const std::string& name) const;

        ManipulationObjectPtr getManipulationObject(const std::string& name);

        std::vector< ManipulationObjectPtr > getManipulationObjects();

        /*!
            Registers the obstacle to this scene. If an obstacle with the same name is already registered nothing happens.
        */
        void registerObstacle(ObstaclePtr obstacle);

        /*!
            Removes the obstacle to from this scene. If the obstacle is not registered nothing happens.
        */
        void deRegisterObstacle(ObstaclePtr obstacle);
        void deRegisterObstacle(const std::string& name);

        bool hasObstacle(ObstaclePtr obstacle) const;
        bool hasObstacle(const std::string& name) const;

        ObstaclePtr getObstacle(const std::string& name);

        std::vector< ObstaclePtr > getObstacles();

        /*!
            Registers the Trajectory to this scene. If an Trajectory with the same name is already registered nothing happens.
        */
        void registerTrajectory(TrajectoryPtr t);

        /*!
            Removes the Trajectory to from this scene. If the Trajectory is not registered nothing happens.
        */
        void deRegisterTrajectory(TrajectoryPtr t);
        void deRegisterTrajectory(const std::string& name);

        bool hasTrajectory(TrajectoryPtr t) const;
        bool hasTrajectory(const std::string& name) const;

        TrajectoryPtr getTrajectory(const std::string& name);

        std::vector< TrajectoryPtr > getTrajectories();
        std::vector< TrajectoryPtr > getTrajectories(const std::string& robotName);

        void registerModelSet(const ModelSetPtr modelSet);
        void deRegisterModelSet(const std::string &name);
        bool hasModelSet(const std::string &name);

        std::vector<ModelSetPtr> getModelSets();
        ModelSetPtr getModelSet(const std::string & name);


        ModelNodeSetPtr getModelNodeSet(const std::string& robot, const std::string rns);

        std::string getName() const;

        /**
         * @param linkVisuType The type of link visualization (e.g. collision).
         * @param visualizationType The name of the VisualizationFactory (@see VisualizationFactory::fromName()) to use.
         *                          If not set, the default VisualizationFactory (@see VisualizationFactory::getGlobalVisualizationFactory()) will be used.
         * @return A visualization of this model's links.
         */
        VisualizationPtr getVisualization(VirtualRobot::ModelLink::VisualizationType linkVisuType = ModelLink::Full, bool addModels = true, bool addObstacles = true, bool addManipulationObjects = true, bool addTrajectories = true, bool addSceneObjectSets = true, std::string visualizationType = "");

        /*!
            Creates an XML string that describes this scene.
            \param basePath All paths to robots or objects are stored relative to this path.
            \return The xml string.
        */
        std::string getXMLString(const std::string& basePath);
    protected:

        std::string name;

        std::vector< RobotPtr > robots;
        std::vector< ObstaclePtr > obstacles;
        std::vector< ManipulationObjectPtr > manipulationObjects;
        std::vector< TrajectoryPtr > trajectories;
        std::vector<ModelSetPtr> modelSets;
    };

} // namespace

#endif // _VirtualRobot_Scene_h_
