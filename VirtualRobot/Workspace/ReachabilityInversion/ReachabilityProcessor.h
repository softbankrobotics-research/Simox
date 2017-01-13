
#ifndef __ReachabilityProcessor_H_
#define __ReachabilityProcessor_H_

#include "../../VirtualRobot.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Obstacle.h>
#include <string.h>
#include <VirtualRobot/IK/PoseQualityExtendedManipulability.h>

#include "OrientedWorkspaceGrid.h"
#include "InverseReachability.h"
#include "RobotPlacementIK.h"
#include "RobotPlacementTrajectoryIK.h"

#include <vector>


namespace VirtualRobot
{
	
class VIRTUAL_ROBOT_IMPORT_EXPORT ReachabilityProcessor
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/*!
		The setup of the IK solver is implictly performed by the data of the reachability files. 
		Here the kinematic chain, the end effector is defined
		Initialize with 
		@param robotFile The simox robot xml file.
		@param reachFile The reachability file.
		@param eef The name of the endeffector as defined in the robot file.
		@param invReachFile The name of the inverse reachability file. Will be created when not present (only when needed)
		@param rnsNameCollisionDetection The RobotNodeSet that should be considered for collision detection with the environment. (optional)
	*/
	ReachabilityProcessor(	const std::string &robotFile, 
							const std::string &reachFile,
							const std::string &eef, 
							const std::string &invReachFile, 
							const std::string &rnsNameCollisionDetection = "",
							const std::string &rnFootL = "", 
							const std::string &rnFootR = "");
	~ReachabilityProcessor();

	/*!
		Performs initialization (i.e. data loading, setup, etc).
	*/
	bool init();

	struct IKResult
	{
		IKResult()
		{
			valid=false;
			poseRobot.setIdentity();
			poseFootLeft.setIdentity();
			poseFootRight.setIdentity();
		}
		bool valid;
		Eigen::Matrix4f target;							// the target that was queried
		Eigen::Matrix4f poseRobot;						// pose of robot (== base coord system)
		Eigen::Matrix4f poseFootLeft;					// pose of left foot
		Eigen::Matrix4f poseFootRight;					// pose of right foot
		VirtualRobot::RobotConfigPtr robotConfig;		// The configuration of the robot (== joint name/angle map)
	};

	/*!
		Perform an IK request. 
		The robot base pose together with a valid configuration is searched.
	*/
	IKResult ik(Eigen::Matrix4f &target);

	std::vector<IKResult> ikTrajectory(std::vector<Eigen::Matrix4f> &target);

	// some getters
	VirtualRobot::RobotPtr getRobot(){return robot;}
	VirtualRobot::RobotNodePtr getFootL(){return footL;}
	VirtualRobot::RobotNodePtr getFootR(){return footR;}
	VirtualRobot::ManipulabilityPtr getReachability(){return reachSpace;}
	OrientedWorkspaceGridPtr getReachGrid(){return reachGrid;}
	InverseReachabilityPtr getInverseReachability(){return invReach;}
	RobotPlacementIKPtr getIKSolver(){return robotPlacementIK;}

	bool isInitialized(){return initialized;}

protected:
	//OrientedWorkspaceGridPtr ReachabilityProcessor::createFullReachGrid(Eigen::Matrix4f &pose);

	float getBaseHeight();

	VirtualRobot::RobotPtr loadRobot(const std::string &robotFile);


	RobotPlacementIKPtr createRobotPlacementIK(bool lazyGridUpdate);
	RobotPlacementTrajectoryIKPtr createRobotPlacementTrajectoryIK(bool lazyGridUpdate);

	VirtualRobot::ManipulabilityPtr loadReachFile(const std::string &filename);

    IKResult doIK(Eigen::Matrix4f &target, bool output, bool considerCollisions = true, int minQuality = -1);
	std::vector<ReachabilityProcessor::IKResult> doIKTrajectory(std::vector<Eigen::Matrix4f> &trajectory, bool output, bool considerCollision = true, int minQuality = -1);

	VirtualRobot::RobotPtr robot;
	VirtualRobot::EndEffectorPtr eef;
	VirtualRobot::RobotNodePtr footL;
	VirtualRobot::RobotNodePtr footR;
	VirtualRobot::ManipulationObjectPtr environment;
	std::string robotFile;
	std::string reachFile;
	std::string invReachFile;
	std::string rnsNameCollisionDetection;
	std::string eefName;
	std::string rnFootL;
	std::string rnFootR;
	VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
	VirtualRobot::SceneObjectSetPtr currentRobotNodeColSet;
	VirtualRobot::ManipulabilityPtr reachSpace;
	bool initialized;

	OrientedWorkspaceGridPtr reachGrid;
	InverseReachabilityPtr invReach;

	RobotPlacementIKPtr robotPlacementIK;
	RobotPlacementTrajectoryIKPtr robotPlacementTrajectoryIK;

	VirtualRobot::PoseQualityExtendedManipulabilityPtr manip;
};

}

#endif // __InvReachScene_WINDOW_H_
