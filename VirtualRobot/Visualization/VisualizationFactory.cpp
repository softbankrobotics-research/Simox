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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#include "VisualizationFactory.h"
#include "../Robot.h"
#include "../SceneObject.h"
#include "../Scene.h"


namespace VirtualRobot
{
    VisualizationPtr VisualizationFactory::getVisualization(RobotPtr robot, VisualizationFactory::VisualizationType visuType, bool sensors)
    {
        if (!robot)
            return VisualizationPtr();

        std::vector<RobotNodePtr> collectedRobotNodes;
        robot->getRobotNodes(collectedRobotNodes);
        std::vector<VisualizationNodePtr> collectedVisualizationNodes;

        for (size_t i = 0; i < collectedRobotNodes.size(); i++)
        {
            collectedVisualizationNodes.push_back(getVisualization(collectedRobotNodes[i], visuType));
        }

        if (sensors)
        {
            std::vector<SensorPtr> sn = robot->getSensors();

            for (size_t i = 0; i < sn.size(); i++)
            {
                collectedVisualizationNodes.push_back(getVisualization(sn[i], visuType));
            }
        }

        return getVisualization(collectedVisualizationNodes);
    }

    VisualizationPtr VisualizationFactory::getVisualization(ScenePtr scene, VisualizationFactory::VisualizationType visuType, bool addRobots, bool addObstacles, bool addManipulationObjects, bool addTrajectories, bool addSceneObjectSets)
    {
        if (!scene)
            return VisualizationPtr();

        std::vector<VisualizationNodePtr> collectedVisualizationNodes;

        if (addRobots)
        {
            std::vector<VirtualRobot::RobotPtr> collectedRobots = scene->getRobots();
            // collect all robotnodes
            std::vector<VirtualRobot::RobotNodePtr> collectedRobotNodes;

            for (size_t i = 0; i < collectedRobots.size(); i++)
            {
                collectedRobots[i]->getRobotNodes(collectedRobotNodes, false);
            }

            for (size_t i = 0; i < collectedRobotNodes.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedRobotNodes[i]->getVisualization(visuType));
            }
        }

        if (addObstacles)
        {
            std::vector<VirtualRobot::ObstaclePtr> collectedObstacles = scene->getObstacles();

            for (size_t i = 0; i < collectedObstacles.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedObstacles[i]->getVisualization(visuType));
            }
        }

        if (addManipulationObjects)
        {
            std::vector<VirtualRobot::ManipulationObjectPtr> collectedManipulationObjects = scene->getManipulationObjects();

            for (size_t i = 0; i < collectedManipulationObjects.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedManipulationObjects[i]->getVisualization(visuType));
            }
        }

        if (addTrajectories)
        {
            std::vector<VirtualRobot::TrajectoryPtr> collectedTrajectories = scene->getTrajectories();

            for (size_t i = 0; i < collectedTrajectories.size(); i++)
            {
                collectedVisualizationNodes.push_back(createTrajectory(collectedTrajectories[i]));
            }
        }

        if (addSceneObjectSets)
        {
            std::vector<VirtualRobot::SceneObjectSetPtr> collectedSceneObjectSets = scene->getSceneObjectSets();

            for (size_t i = 0; i < collectedSceneObjectSets.size(); i++)
            {
                std::vector< SceneObjectPtr > sos = collectedSceneObjectSets[i]->getSceneObjects();

                for (size_t j = 0; j < sos.size(); j++)
                {
                    collectedVisualizationNodes.push_back(sos[j]->getVisualization(visuType));
                }
            }
        }

        return getVisualization(collectedVisualizationNodes);
    }

    VisualizationNodePtr VisualizationFactory::getVisualization(SceneObjectPtr so, VisualizationFactory::VisualizationType visuType)
    {
        return so->getVisualization(visuType);
    }
} // namespace VirtualRobot

