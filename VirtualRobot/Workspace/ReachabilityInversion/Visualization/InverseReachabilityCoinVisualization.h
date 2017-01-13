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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_InverseReachabilityCoinVisu_h_
#define _VirtualRobot_InverseReachabilityCoinVisu_h_

#include "../../../VirtualRobot.h"
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/OrientedWorkspaceGrid.h>
#include <VirtualRobot/Workspace/ReachabilityInversion/InverseReachability.h>
#include <VirtualRobot/Workspace/Manipulability.h>

namespace VirtualRobot
{

/*!
	Some helper methods to visualize reachability data
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT InverseReachabilityCoinVisualization
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	static SoNode* getCoinVisualization( OrientedWorkspaceGridPtr reachGrid, VirtualRobot::ColorMap cm, bool transformToGlobalPose = true );

	static void buildRobotVisu(SoSeparator* robotVisuSep, VirtualRobot::RobotPtr robot);
	static void buildObjectVisu(SoSeparator *objectVisuSep, bool enable, bool showFloor, VirtualRobot::ObstaclePtr graspObject, VirtualRobot::ObstaclePtr environment=VirtualRobot::ObstaclePtr(), Eigen::Vector3f floorpos = Eigen::Vector3f(0,0,0));
	static void buildReachVisu(SoSeparator* reachabilityVisuSep, bool enable, InverseReachabilityPtr invReach, Eigen::Matrix4f &tragetPose);
	static void buildReachMapVisu(SoSeparator* reachabilityMapVisuSep, bool enable, InverseReachabilityPtr invReach, OrientedWorkspaceGridPtr reachGrid, SoSeparator* reachableFootVisuSep, SoSeparator *footLRVisuSep);

	static void addFootstepVisu(SoSeparator* s, Eigen::Matrix4f &gp);

	static SoNode* getCoinVisualization( OrientedWorkspaceGridPtr reachGrid, InverseReachabilityPtr invReach, VirtualRobot::ColorMap cm, bool transformToGlobalPose, SoSeparator* reachableFootVisuSep, SoSeparator *footLRVisuSep );
	static SoNode* getCoinVisualization2( OrientedWorkspaceGridPtr reachGrid, InverseReachabilityPtr invReach, VirtualRobot::ColorMap cm, bool transformToGlobalPose = true );
	static SoNode* getCoinVisualizationCut(VirtualRobot::WorkspaceRepresentationPtr reachSpace, const VirtualRobot::ColorMap cm, bool transformToGlobalPose);

    /*!
      * Creates the 2d polygon visu of a foot (or a base node), which is generated as the convex hull of the projection of the collision model on the floor.
      * \param visuFootSep Store the visualization here, adds a separator with the visu of the foot (or the base)
      * \param footNode The robot node that contains the collision model of the foot/the base which should be used
      * \param reachSpace The transformations are determined from here
      * \param trafoBaseToFoot The transformation is stored here
      * */
    static void buildFootPolygon(SoSeparator *visuFootSep, VirtualRobot::RobotNodePtr footNode, VirtualRobot::ManipulabilityPtr reachSpace, Eigen::Matrix4f &trafoBaseToFoot);

    static void buildFeetPolygons(SoSeparator *footLRVisuSep, VirtualRobot::RobotNodePtr footL, VirtualRobot::RobotNodePtr footR, VirtualRobot::ManipulabilityPtr reachSpace,Eigen::Matrix4f &trafoBaseToFootL, Eigen::Matrix4f &trafoBaseToFootR);
	static void addFootstepVisu(SoSeparator* s, SoSeparator* footLRVisuSep, Eigen::Matrix4f &gp);

protected:

	InverseReachabilityCoinVisualization(){};

};

} // namespace VirtualRobot

#endif // _VirtualRobot_InverseReachabilityCoinVisu_h_
