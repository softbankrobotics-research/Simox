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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/CollisionDetection/CollisionModelImplementation.h>
#include <VirtualRobot/CollisionDetection/CollisionCheckerImplementation.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/EndEffector/EndEffectorActor.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/Model/Nodes/ModelJointPrismatic.h>
#include <VirtualRobot/Model/Nodes/ModelJointRevolute.h>
#include <VirtualRobot/Model/Nodes/ModelJointFixed.h>
#include <VirtualRobot/Tools/ConditionedLock.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/Visualization.h>
#include <VirtualRobot/Visualization/ColorMap.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/BaseIO.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/IK/IKSolver.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <VirtualRobot/IK/PoseQualityMeasurement.h>
#include <VirtualRobot/IK/PoseQualityManipulability.h>
#include <VirtualRobot/Workspace/WorkspaceDataArray.h>
#include <VirtualRobot/Workspace/WorkspaceData.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/VoxelTree6D.hpp>
#include <VirtualRobot/Workspace/VoxelTree6DElement.hpp>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/Grasping/BasicGraspQualityMeasure.h>
#include <VirtualRobot/AbstractFactoryMethod.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/ModelConfig.h>
#include <VirtualRobot/Model/ModelNodeSet.h>
#include <VirtualRobot/Trajectory.h>
#include <VirtualRobot/Model/ModelFactory.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/VirtualRobotImportExport.h>
//#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Model/ManipulationObject.h>
#include <VirtualRobot/Tools/BoundingBox.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Compression/CompressionRLE.h>
#include <VirtualRobot/Compression/CompressionBZip2.h>
#include <VirtualRobot/Tools/SphereApproximator.h>

