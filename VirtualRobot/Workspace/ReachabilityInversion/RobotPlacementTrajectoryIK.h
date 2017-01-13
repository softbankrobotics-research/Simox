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
#ifndef _VirtualRobot_RobotPlacementTrajectory_h_
#define _VirtualRobot_RobotPlacementTrajectory_h_

#include "../../VirtualRobot.h"
#include  <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include  <VirtualRobot/VirtualRobot.h>
#include  <VirtualRobot/IK/GenericIKSolver.h>
#include  "InverseReachability.h"
#include  "OrientedWorkspaceGrid.h"
#include  "RobotPlacementIK.h"


namespace VirtualRobot
{

/*!
	This IK solver can be used to find a suitable robot pose (x,y,alpha on the floor) for execution of a workspace trajectory.
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT RobotPlacementTrajectoryIK : public RobotPlacementIK, public boost::enable_shared_from_this<RobotPlacementTrajectoryIK>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		Initialize the IK solver
		\param baseNode The node which should be aligned with the data of invReach. This means that the robot's global pose is adjusted, so that the frame of this node is aligned with the floor-projected data of the inverse reachability.
		\param rns The kinematic chain for IK solving
		\param invReach The oriented reachability map. This data is used to build the oriented reachability map on the floor w.r.t. the grasping pose.
        \param lazyGridUpdate If set, the reachability grid entries are computed on the fly.
	*/
	RobotPlacementTrajectoryIK(VirtualRobot::RobotNodePtr baseNode, VirtualRobot::RobotNodeSetPtr rns, InverseReachabilityPtr invReach, bool lazyGridUpdate = false);

	struct IkSolution
	{
		IkSolution()
		{
			valid = false;
			quality = 0;
		}
		std::vector<float> jointValues;
		std::string rns;
		std::string tcp;
		bool valid;
		float quality;
	};

	struct TrajectoryIK
	{
		TrajectoryIK()
		{
			valid = false;
			x=y=alpha=0;
			robotPose = Eigen::Matrix4f::Identity();
		}
		bool valid;										//!< IK request was solved
		Eigen::Matrix4f robotPose;						//!< Resulting robot base pose
		float x,y,alpha;								//!< Resulting robot base pose as x,y,alpha values
		std::vector< Eigen::Matrix4f > wsTrajectory;	//!< The requested trajectory in workspace
		std::vector< IkSolution >  jsTrajectory;		//!< The IK solution in joint space
	};

	/*!
	    This method tries to solve the extended IK query that consists of
			* placing the robot according to the inverse reachability information
			* solve the IK for all workspace poses
	    \param trajectory The workspace trajectory given in global coordinate system.
	    \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
		\param maxLoops How often should we try.
	    \return The robot pose and the joint space trajectory (TrajectoryIK.valid is true when a solution was found)
	*/
	virtual TrajectoryIK solve(const std::vector< Eigen::Matrix4f > &trajectory, CartesianSelection selection = All, int maxLoops = 1);


	/*!
		This method tries to solve the extended IK query that consists of
			* placing the robot according to the inverse reachability information
			* solve the IK for all workspace poses
		\param reachGrid Use a custom reachability grid for robot placement.
		\param trajectory The workspace trajectory given in global coordinate system.
		\param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
		\param maxLoops How often should we try.
		\return The robot pose and the joint space trajectory (TrajectoryIK.valid is true when a solution was found)
	*/	
	virtual TrajectoryIK solve(OrientedWorkspaceGridPtr reachGrid, const std::vector< Eigen::Matrix4f > &trajectory, CartesianSelection selection = All, int maxLoops = 1);


	/*!
		Just sets the robot base pose according to the inverse reachability data.
		\param trajectory The trajectory that should be reachable after the robot is positioned.
	*/
	virtual bool setRobotPoseTargetReachable(const std::vector< Eigen::Matrix4f > &trajectory, int maxLoops = 5);

    OrientedWorkspaceGridPtr getReachGrid();
	
	/*!
		Search a suitable robot pose according to the reachability map.
		\param reachGrid The reachability map to query for robot poses
		\param minEntry The minimum entry that must be achieved (e.g. use 1 here)
		\param bestEntry If a pose with this entry is sampled it is returned
		\param maxLoops How often should we sample until we return the best achieved pose. If in between a pose with bestEntry is sampled the loop is stopped earlier.
		\return True if at least minEntry could have been achieved. The robot is placed at the resulting pose (x,y, alpha).
	*/
	bool searchRobotPose(OrientedWorkspaceGridPtr reachGrid, int minEntry, int bestEntry, int maxLoops, float &storeX, float &storeY, float &storeAlpha);
protected:

	bool updateReachGrid(const std::vector< Eigen::Matrix4f > &trajectory);
	bool trySolve();
	bool doIK();
	void setJointsZero();
	std::vector< Eigen::Matrix4f > currentTrajectory;
};


typedef boost::shared_ptr<RobotPlacementTrajectoryIK> RobotPlacementTrajectoryIKPtr;
} // namespace VirtualRobot

#endif // _VirtualRobot_RobotPlacementTrajectory_h_

