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
#ifndef _VirtualRobot_RobotPlacementIK_h_
#define _VirtualRobot_RobotPlacementIK_h_

#include "../../VirtualRobot.h"
#include  <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include  <VirtualRobot/VirtualRobot.h>
#include  <VirtualRobot/IK/GenericIKSolver.h>
#include  "InverseReachability.h"
#include  "OrientedWorkspaceGrid.h"


namespace VirtualRobot
{

/*!
	This IK solver can be used to find IK solutions to the extended IK problem that consists of finding suitable robot base positions together with an configuration of the arm.
	Hence, the globalpose of the robot is not assumed to be fixed, but an inverse reachability distribution is used to query promising robot base poses in the world in order to apply a grasp.
	Thereby the robot is place on the floor (x,y plane, rotation around z axis).
*/
class VIRTUAL_ROBOT_IMPORT_EXPORT RobotPlacementIK : public VirtualRobot::GenericIKSolver, public boost::enable_shared_from_this<RobotPlacementIK>
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
	RobotPlacementIK(VirtualRobot::RobotNodePtr baseNode, VirtualRobot::RobotNodeSetPtr rns, InverseReachabilityPtr invReach, bool lazyGridUpdate = false);

	/*!
	    This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
		An inverse reachability grid is built according to the grasping pose and the robot is positioned according to the inverse reachability data.
	    \param globalPose The target pose given in global coordinate system.
	    \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
		\param maxLoops How often should we try.
		\param bestEntry This parameter is used to search a robot pose in the reach grid. If not set, 75% of the max entry is chosen.
	    \return true on success
	*/
	virtual bool solve(const Eigen::Matrix4f &globalPose, CartesianSelection selection = All, int maxLoops = 1, int bestEntry = -1);


	/*!
	    This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
		The robot is positioned according to the inverse reachability grid data.
		\param reachGrid Use a custom reachability grid for robot placement.
	    \param globalPose The target pose given in global coordinate system.
	    \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
		\param maxLoops How often should we try.
		\param bestEntry This parameter is used to search a robot pose in the reach grid. If <=0, 75% of the max entry is chosen.
	    \return true on success
	*/	
	virtual bool solve(OrientedWorkspaceGridPtr reachGrid, const Eigen::Matrix4f &globalPose, CartesianSelection selection = All, int maxLoops = 1, int bestEntry = -1);


	/*!
		Just sets the robot base pose according to the inverse reachability data.
		\param globalPose The pose that should be reachable after the robot is positioned.
	*/
	virtual bool setRobotPoseTargetReachable(const Eigen::Matrix4f &globalPose, int maxLoops = 5);


	/*!
	    This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
		\param object The grasps of this object are checked if the stored TCP is identical with teh TCP of teh current RobotNodeSet, and the an IK solution for one of remaining grasps is searched.
	    \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
		\param maxLoops How often should we try.
	    \return On success: The grasp for which an IK-solution was found, otherwise an empty GraspPtr
	*/
	//virtual GraspPtr solve(ManipulationObjectPtr object, CartesianSelection selection = All, int maxLoops = 1);
	//virtual bool solve(ManipulationObjectPtr object, GraspPtr grasp, CartesianSelection selection = All, int maxLoops = 1);
	    
    OrientedWorkspaceGridPtr getReachGrid();
	struct PerformanceMeasure
	{
		PerformanceMeasure()
		{
			reset();
		}
		void reset()
		{
			collisionCount = 0;
			robotPlacementCount = 0;
			ikRequestsSuccess = 0;
			ikRequestsFailed = 0;
			queryTimeAccumulatedAll = 0;
			queryTimeAccumulatedSuccess = 0;
			queryTimeReachGrid = 0;
			queryTimeIKSolverArm = 0;
            setupTime = 0;
            setupCount = 0;
			reachGridSuccess = 0;
			reachGridFailed = 0;
			ikSolverArmSuccess = 0;
			ikSolverArmFailed = 0;

		}

		void print()
		{
			std::cout << "** RobotPlacementIK performance measurement **" << std::endl;
			std::cout << " * IK requests: " << ikRequestsSuccess+ikRequestsFailed << "(success:" << ikRequestsSuccess << ", failed:" << ikRequestsFailed << ")" << std::endl;
			std::cout << " * IK success rate = ";
			if (ikRequestsSuccess+ikRequestsFailed!=0)
				std::cout << float(ikRequestsSuccess) / float(ikRequestsSuccess+ikRequestsFailed);
			std::cout << std::endl;
			std::cout << " * collisions detected: " << collisionCount << std::endl;
			std::cout << " * robot places generated: " << robotPlacementCount << std::endl;
			float avgQueryTime = -1.0f;
			float avgQueryTimeAll = -1.0f;
			if (ikRequestsSuccess!=0)
				avgQueryTime = queryTimeAccumulatedSuccess / float(ikRequestsSuccess);
			if (ikRequestsSuccess+ikRequestsFailed!=0)
				avgQueryTimeAll = queryTimeAccumulatedAll / float(ikRequestsSuccess+ikRequestsFailed);

			std::cout << " * avg IK query time (all):" << avgQueryTimeAll << "ms" << std::endl;
            std::cout << " * avg IK query time (only success):" << avgQueryTime << "ms" << std::endl;

			std::cout << " * setup counts (new target pose): " << setupCount << std::endl;
			avgQueryTime = -1.0f;
			if (setupCount!=0)
				avgQueryTime = setupTime / float(setupCount);
			std::cout << " * avg setup time:" << avgQueryTime << "ms" << std::endl;

			std::cout << " * reach grid queries: " << reachGridSuccess+reachGridFailed << " (success:" << reachGridSuccess << ", failed: " <<reachGridFailed << ")" << std::endl;
			avgQueryTime = -1.0f;
			if (reachGridSuccess+reachGridFailed!=0)
				avgQueryTime = queryTimeReachGrid / float(reachGridSuccess+reachGridFailed);
			std::cout << " * avg reach grid query time:" << avgQueryTime << "ms" << std::endl;
			std::cout << " * accumulated reach grid time: :" << avgQueryTime*(reachGridSuccess+reachGridFailed) << "ms" << std::endl;

			std::cout << " * ik solver arm queries: " << ikSolverArmFailed+ikSolverArmSuccess << " (success:" << ikSolverArmSuccess << ", failed: " << ikSolverArmFailed << ")" << std::endl;
			avgQueryTime = -1.0f;
			if (ikSolverArmFailed+ikSolverArmSuccess!=0)
				avgQueryTime = queryTimeIKSolverArm / float(ikSolverArmFailed+ikSolverArmSuccess);
			std::cout << " * avg ik solver arm query time:" << avgQueryTime << "ms" << std::endl;
			std::cout << " * accumulated IK time: :" << avgQueryTime*(ikSolverArmFailed+ikSolverArmSuccess) << "ms" << std::endl;


		}
		// some variables for performance evaluation
		int collisionCount;
		int robotPlacementCount;
		int ikRequestsSuccess;
		int ikRequestsFailed;
		float queryTimeAccumulatedSuccess;
		float queryTimeAccumulatedAll;
		float queryTimeReachGrid;
		float queryTimeIKSolverArm;
        float setupTime;
        int setupCount;
		int reachGridSuccess;
		int reachGridFailed;
		int ikSolverArmSuccess;
		int ikSolverArmFailed;

	};

	void resetPerformanceMeasure(){performanceMeasure.reset();}
	void printPerformanceMeasure(){performanceMeasure.print();}
	PerformanceMeasure getPerformanceMeasure(){return performanceMeasure;}

	/*!
		Search a suitable robot pose according to the reachability map.
		\param reachGrid The reachability map to query for robot poses
		\param minEntry The minimum entry that must be achieved (e.g. use 1 here)
		\param bestEntry If a pose with this entry is sampled it is returned
		\param maxLoops How often should we sample until we return the best achieved pose. If in between a pose with bestEntry is sampled the loop is stopped earlier.
		\return True if at least miNEntry could have been achieved. The robot is placed at the resulting pose (x,y, alpha).
	*/
	bool searchRobotPose(OrientedWorkspaceGridPtr reachGrid, int minEntry, int bestEntry, int maxLoops);
protected:

	//void setGlobalPoseForRobotNode(VirtualRobot::RobotNodePtr rn, Eigen::Matrix4f &gp);
	bool updateReachGrid(const Eigen::Matrix4f &globalPose);
	bool trySolve();

	bool doIK();

	void setJointsZero();

	VirtualRobot::RobotNodePtr baseNode;
	InverseReachabilityPtr invReach;
	Eigen::Matrix4f currentTargetPose;
	OrientedWorkspaceGridPtr reachGrid;
	VirtualRobot::RobotPtr robot;
    bool lazyGridUpdate;

	float height;

	PerformanceMeasure performanceMeasure;

};


typedef boost::shared_ptr<RobotPlacementIK> RobotPlacementIKPtr;
} // namespace VirtualRobot

#endif // _Reachability_h_
