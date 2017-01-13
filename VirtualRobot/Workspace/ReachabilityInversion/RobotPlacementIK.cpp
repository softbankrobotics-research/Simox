#include "RobotPlacementIK.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <fstream>
#include <cmath>
#include <float.h>
#include <limits.h>

using namespace std;

namespace VirtualRobot {

RobotPlacementIK::RobotPlacementIK(VirtualRobot::RobotNodePtr baseNode, VirtualRobot::RobotNodeSetPtr rns, InverseReachabilityPtr invReach, bool lazyGridUpdate)
	:VirtualRobot::GenericIKSolver(rns),
	baseNode(baseNode),
	invReach(invReach)
{
	VR_ASSERT(baseNode);
	VR_ASSERT(invReach);
	robot = baseNode->getRobot();
    // assume that current height of base node is fixed (in this height the inv reach data is cut in order to generate the reachGrid)
	height = baseNode->getGlobalPose()(2,3);
	VR_ASSERT(robot);
    this->lazyGridUpdate = lazyGridUpdate;
	float tr = invReach->getDiscretizeParameterTranslation();
	float rot = invReach->getDiscretizeParameterRotation();
	cout << "Discretization trans:" << tr << ", rot:" << rot << endl;
	// todo: adjust extends according to max possible invReach extends!!!
	float minX = invReach->getMinBound(0);
	float maxX = invReach->getMaxBound(0);
	float minY = invReach->getMinBound(1);
	float maxY = invReach->getMaxBound(1);
	reachGrid.reset(new OrientedWorkspaceGrid(minX,maxX,minY,maxY,tr,rot,lazyGridUpdate,height));
	verbose = true;
}

bool RobotPlacementIK::solve( const Eigen::Matrix4f &globalPose, CartesianSelection selection /*= All*/, int maxLoops /*= 1*/, int bestEntry )
{
	if (!updateReachGrid(globalPose))
	{
		VR_ERROR << "Could not update reach grid..." << endl;
		return false;
	}
	return solve (reachGrid,globalPose,selection,maxLoops,bestEntry);
}

bool RobotPlacementIK::solve( OrientedWorkspaceGridPtr reachGrid, const Eigen::Matrix4f &globalPose, CartesianSelection selection /*= All*/, int maxLoops /*= 1*/, int bestEntry )
{
	clock_t startTime = clock();
	jacobian->setGoal(globalPose,tcp,selection,maxErrorPositionMM,maxErrorOrientationRad);
	jacobian->checkImprovements(true);
	if (bestEntry<=0)
	{
		bestEntry = int (float(reachGrid->getMaxEntry()) * 0.75f);
		if (bestEntry<1)
			bestEntry = 1;
	}
	int minEntry = 1;

	std::vector<float> jv = rns->getJointValues();


	for (int i=1; i<maxLoops; i++)
	{
		if (!searchRobotPose(reachGrid,minEntry,bestEntry,maxLoops*10))
			continue;

		// check reachability-> not needed
		//if (!checkReachable(globalPose))
		//	return false;

		if (trySolve())
		{
			clock_t endTime = clock();
			float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
			performanceMeasure.queryTimeAccumulatedAll += timeMS;
			performanceMeasure.queryTimeAccumulatedSuccess += timeMS;
			performanceMeasure.ikRequestsSuccess++;
			return true;
		}
		
		rns->setJointValues(jv);
		//if (i<maxLoops-1)
			//setJointsRandom(); // todo: check if we should reset the joints to standard pose -> better results?
	}
	clock_t endTime = clock();
	float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
	performanceMeasure.queryTimeAccumulatedAll += timeMS;
	performanceMeasure.ikRequestsFailed++;

	return false;
}

bool RobotPlacementIK::updateReachGrid( const Eigen::Matrix4f &globalPose )
{
	if (globalPose == currentTargetPose && reachGrid)
		return true;
    clock_t startTime = clock();
    performanceMeasure.setupCount++;
	currentTargetPose = globalPose;
    bool res;
    if (lazyGridUpdate)
        res = reachGrid->fillDataLazy(globalPose, invReach);
    else
        res = reachGrid->fillData(globalPose, invReach);
    clock_t endTime = clock();
    float timeMS = float(endTime-startTime) / float(CLOCKS_PER_SEC) * 1000.0f;
    performanceMeasure.setupTime += timeMS;
    return res;
}
/*
void RobotPlacementIK::setGlobalPoseForRobotNode(RobotNodePtr rn, Eigen::Matrix4f &gp)
{
	cout << "goal\n" << gp << endl;
	cout << "current\n" << rn->getGlobalPose() << endl;

		// get transformation from current to wanted tcp pose
	Eigen::Matrix4f t = gp * rn->getGlobalPose().inverse();

	cout << "t\n" << t << endl;

		// apply transformation to current global pose of robot
	Eigen::Matrix4f	t2 = robot->getGlobalPose() *t;
	cout << "t2\n" << t2 << endl;

		// set t2
		robot->setGlobalPose(t2);
		cout << "result\n" << rn->getGlobalPose() << endl;
}*/

bool RobotPlacementIK::searchRobotPose( OrientedWorkspaceGridPtr reachGrid, int minEntry, int bestEntry, int maxLoops )
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
				{
					break;
				}
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

bool RobotPlacementIK::setRobotPoseTargetReachable( const Eigen::Matrix4f &globalPose, int maxLoops )
{
	if (!updateReachGrid(globalPose))
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


	for (int i=1; i<maxLoops; i++)
	{
		if (searchRobotPose(reachGrid,minEntry,bestEntry,maxLoops*10))
			return true;
	}
	return false;
}

OrientedWorkspaceGridPtr RobotPlacementIK::getReachGrid()
{
    return reachGrid;
}


void RobotPlacementIK::setJointsZero()
{
	std::vector<float> jv(rns->getSize(),0.0f);
	rns->setJointValues(jv);  
}


bool RobotPlacementIK::trySolve()
{
	setJointsZero();
	if (doIK())
	{
		// we found a solution
		return true; 
	}
	
	// check if a random config works better
	setJointsRandom();
	return doIK();
}



bool RobotPlacementIK::doIK()
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
