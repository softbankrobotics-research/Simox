#include "RobotPlacementTrajectoryIK.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

using namespace std;

namespace VirtualRobot {

RobotPlacementTrajectoryIK::RobotPlacementTrajectoryIK(VirtualRobot::RobotNodePtr baseNode, VirtualRobot::RobotNodeSetPtr rns, InverseReachabilityPtr invReach, bool lazyGridUpdate)
	:RobotPlacementIK(baseNode,rns,invReach,lazyGridUpdate)
{
	verbose = true;
}

RobotPlacementTrajectoryIK::TrajectoryIK RobotPlacementTrajectoryIK::solve( const std::vector< Eigen::Matrix4f > &trajectory, CartesianSelection selection /*= All*/, int maxLoops /*= 1*/ )
{
	if (!updateReachGrid(trajectory))
	{
		VR_ERROR << "Could not update reach grid..." << endl;
		TrajectoryIK r;
		r.valid = false;
		return r;
	}
	return solve (reachGrid,trajectory,selection,maxLoops);
}

RobotPlacementTrajectoryIK::TrajectoryIK RobotPlacementTrajectoryIK::solve( OrientedWorkspaceGridPtr reachGrid, const std::vector< Eigen::Matrix4f > &trajectory, CartesianSelection selection /*= All*/, int maxLoops /*= 1*/ )
{
	clock_t startTime = clock();
	
	int bestEntry = int (float(reachGrid->getMaxEntry()) * 0.75f);
	if (bestEntry<1)
		bestEntry = 1;
	int minEntry = 1;

	TrajectoryIK result;
	result.wsTrajectory = trajectory;

	std::vector<float> jv = rns->getJointValues();
	if (verbose)
		VR_INFO << "Searching IK solution" << endl;


	for (int i=1; i<maxLoops; i++)
	{
		if (verbose)
			VR_INFO << "Loop " << i << endl;

		if (!searchRobotPose(reachGrid,minEntry,bestEntry,maxLoops*10,result.x,result.y,result.alpha))
			continue;
		if (verbose)
			VR_INFO << "Found robot pose" << endl;

		// check reachability-> not needed
		//if (!checkReachable(globalPose))
		//	return false;
		result.jsTrajectory.clear();
		bool ikOK = true;
		setJointsZero();
		for (size_t tr=0;tr<trajectory.size();tr++)
		{
			if (verbose)
				VR_INFO << "Checking tr point " << tr;

			jacobian->setGoal(trajectory[tr],tcp,selection,maxErrorPositionMM,maxErrorOrientationRad);
			jacobian->checkImprovements(true);

			if (trySolve())
			{
				if (verbose)
					cout << ".. ok" << endl;
				IkSolution s;
				s.valid = true;
				s.quality = 0;
				s.rns = rns->getName();
				s.tcp = rns->getTCP()->getName();
				s.jointValues = rns->getJointValues();
				VR_ASSERT(s.jointValues.size() == rns->getSize());
				result.jsTrajectory.push_back(s);

			} else
			{
				if (verbose)
					cout << ".. failed" << endl;
				ikOK = false;
				break;
			}
		}
		if (ikOK)
		{
			if (verbose)
				cout << ".. found IK solution" << endl;
			clock_t endTime = clock();
			float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
			performanceMeasure.queryTimeAccumulatedAll += timeMS;
			performanceMeasure.queryTimeAccumulatedSuccess += timeMS;
			performanceMeasure.ikRequestsSuccess++;
			result.valid = true;
			result.robotPose = robot->getGlobalPose();
			return result;
		}

		
		rns->setJointValues(jv);
		//if (i<maxLoops-1)
			//setJointsRandom(); // todo: check if we should reset the joints to standard pose -> better results?
	}
	if (verbose)
		cout << "IK search failed " << endl;
	clock_t endTime = clock();
	float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
	performanceMeasure.queryTimeAccumulatedAll += timeMS;
	performanceMeasure.ikRequestsFailed++;
	result.valid = false;
	return result;
}

bool RobotPlacementTrajectoryIK::updateReachGrid( const std::vector< Eigen::Matrix4f > &trajectory )
{
	// check if trajectory changed
	if (reachGrid && trajectory.size()==currentTrajectory.size())
	{
		bool ok = true;
		for (size_t i=0;i<trajectory.size();i++)
		{
			if (trajectory[i] != currentTrajectory[i])
			{
				ok=false;
				break;
			}
		}
		if (ok)
			return true;
	}
	if (verbose)
		VR_INFO << "Updating reach grid" << endl;
    clock_t startTime = clock();
    performanceMeasure.setupCount++;
	currentTrajectory = trajectory;
    bool res;
    if (lazyGridUpdate)
        res = reachGrid->fillDataLazy(trajectory, invReach);
    else
        res = reachGrid->fillData(trajectory, invReach);
    clock_t endTime = clock();
    float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
    performanceMeasure.setupTime += timeMS;
    return res;
}

bool RobotPlacementTrajectoryIK::searchRobotPose( OrientedWorkspaceGridPtr reachGrid, int minEntry, int bestEntry, int maxLoops, float &storeX, float &storeY, float &storeAlpha )
{
	if (!reachGrid)
		return false;
	clock_t startTime = clock();

	Eigen::Matrix4f origGP = robot->getGlobalPose();
	
	int actBest = 0;
	float actX = 0.0f;
	float actY = 0.0f;
	float actAlpha = 0.0f;
	GraspPtr g;
	for (int i=0;i<maxLoops;i++)
	{
		float x,y,a;
		int e;
		if (reachGrid->getRandomPos(minEntry,x,y,a,e,g))
		{
			if (e>actBest)
			{
				actX = x;
				actY = y;
				actAlpha = a;
				actBest = e;
				if (actBest>=bestEntry)
					break;
			}
		}
	}
	if (actBest>=minEntry)
	{
		if (verbose)
			VR_INFO << "Robot pose:" << actX << "," << actY << ", " << actAlpha << ", entry:" << actBest << endl;

		Eigen::Matrix4f gp;
		float posrpy[6];
		posrpy[0] = actX;
		posrpy[1] = actY;
		posrpy[2] = reachGrid->getHeight();
		posrpy[3] = 0.0f;
		posrpy[4] = 0.0f;
		posrpy[5] = actAlpha;
		MathTools::posrpy2eigen4f(posrpy,gp); // roll,pitch are zero -> RPY == EulerXYZ
		robot->setGlobalPoseForRobotNode(baseNode,gp);
		if (cdm && cdm->isInCollision())
		{
			performanceMeasure.collisionCount++;
			if (verbose)
				cout << "Collision..." << endl;
		} else
		{
			clock_t endTime = clock();
			float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
			performanceMeasure.queryTimeReachGrid += timeMS;
			performanceMeasure.reachGridSuccess++;
			performanceMeasure.robotPlacementCount++;
			storeX = actX;
			storeY = actY;
			storeAlpha = actAlpha;
			return true;
		}
	}
	robot->setGlobalPose(origGP);
	clock_t endTime = clock();
	float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
	performanceMeasure.queryTimeReachGrid += timeMS;
	performanceMeasure.reachGridFailed++;
	return false;
}

bool RobotPlacementTrajectoryIK::setRobotPoseTargetReachable( const std::vector< Eigen::Matrix4f > &trajectory, int maxLoops )
{
	if (!updateReachGrid(trajectory))
	{
		VR_ERROR << "Could not update reach grid..." << endl;
		return false;
	}
	int bestEntry;
    if (lazyGridUpdate)
        bestEntry =  int(float(invReach->getMaxEntry()) * 0.75f); // use inv reach best entry, since we don't know about the best entry of the grid yet
    else
        bestEntry = int(float(reachGrid->getMaxEntry()) * 0.75f);

	if (bestEntry<1)
		bestEntry = 1;
	int minEntry = 1;

	float x,y,alpha;


	for (int i=1; i<maxLoops; i++)
	{
		if (searchRobotPose(reachGrid,minEntry,bestEntry,maxLoops*10,x,y,alpha))
			return true;
	}
	return false;
}

OrientedWorkspaceGridPtr RobotPlacementTrajectoryIK::getReachGrid()
{
    return reachGrid;
}

void RobotPlacementTrajectoryIK::setJointsZero()
{
	std::vector<float> jv(rns->getSize(),0.0f);
	rns->setJointValues(jv);  
}

bool RobotPlacementTrajectoryIK::trySolve()
{
	if (doIK())
	{
		// we found a solution
		return true; 
	}

	// check if a random config works better
	setJointsRandom();
	return doIK();
}



bool RobotPlacementTrajectoryIK::doIK()
{
	clock_t startTime = clock();
	bool res = jacobian->solveIK(jacobianStepSize,0.0,jacobianMaxLoops);
	
	if (res)
	{
		if (cdm)
		{
			if (!cdm->isInCollision())
			{
				clock_t endTime = clock();
				float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
				performanceMeasure.queryTimeIKSolverArm += timeMS;
				performanceMeasure.ikSolverArmSuccess++;
				if (verbose)
					VR_INFO << "IK success.." << endl;
				return true;
			} else
			{
				if (verbose)
					VR_INFO << "IK solution in collision.." << endl;
				performanceMeasure.collisionCount++;
			}
		} else
		{
			clock_t endTime = clock();
			float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
			performanceMeasure.queryTimeIKSolverArm += timeMS;
			performanceMeasure.ikSolverArmSuccess++;
			if (verbose)
				VR_INFO << "IK success.." << endl;
			return true;
		}
	}
	clock_t endTime = clock();
	float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
	performanceMeasure.queryTimeIKSolverArm += timeMS;
	performanceMeasure.ikSolverArmFailed++;
	if (verbose)
		VR_INFO << "IK failed.." << endl;
	return false;
}

}
