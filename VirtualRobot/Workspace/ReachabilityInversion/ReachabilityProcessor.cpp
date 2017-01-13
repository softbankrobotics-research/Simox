
#include "ReachabilityProcessor.h"
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Workspace/WorkspaceGrid.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Trajectory.h>

#include <Eigen/Geometry>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <sstream>
using namespace std;
using namespace Eigen;

namespace VirtualRobot
{

ReachabilityProcessor::ReachabilityProcessor(	const std::string &robotFile, 
												const std::string &reachFile,
												const std::string &eef, 
												const std::string &invReachFile, 
												const std::string &rnsNameCollisionDetection,
												const std::string &rnFootL, 
												const std::string &rnFootR)
{
	this->robotFile = robotFile;
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(this->robotFile);
	this->rnsNameCollisionDetection = rnsNameCollisionDetection;
	this->reachFile = reachFile;
	this->eefName = eef;
	this->invReachFile = invReachFile;
	this->rnsNameCollisionDetection = rnsNameCollisionDetection;
	this->rnFootL = rnFootL;
	this->rnFootR = rnFootR;
	initialized = false;
}


ReachabilityProcessor::~ReachabilityProcessor()
{
}


bool ReachabilityProcessor::init()
{
	initialized = false;
	VR_INFO << "** START INIT" << endl;
	VR_INFO << "** LOADING ROBOT" << endl;
	robot = loadRobot(robotFile);
	if (!robot)
	{
		VR_ERROR << "Error loading robot. Aborting" << endl;
		return false;
	}

	if (!rnFootL.empty() && !rnFootR.empty())
	{
		if (robot->hasRobotNode(rnFootL))
			footL = robot->getRobotNode(rnFootL);
		else
		{
			VR_WARNING << "Could not find footL:" << rnFootL << endl;
		}
		if (robot->hasRobotNode(rnFootR))
			footR = robot->getRobotNode(rnFootR);
		else
		{
			VR_WARNING << "Could not find footR:" << rnFootR << endl;
		}
	}

	VR_INFO << "** LOADING REACHABILITY DATA" << endl;
	reachSpace = loadReachFile(reachFile);
	if (!reachSpace)
	{
		VR_ERROR << "Could not init reachability data, file:" << reachFile << endl;
		return false;
	}
	reachSpace->print();
	currentRobotNodeSet = reachSpace->getNodeSet();
	currentRobotNodeColSet = reachSpace->getCollisionModelDynamic();


	VR_INFO << "** LOADING INVERSE REACHABILITY DATA" << endl;
	if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(invReachFile))
	{
		invReach.reset(new InverseReachability(robot,invReachFile));
		invReach->print();
	} else
	{
		cout << "Building INV reach file and saving to " << invReachFile << endl;
		invReach.reset(new InverseReachability(reachSpace));
		if (!invReachFile.empty())
			invReach->save(invReachFile);
	}

	eef = robot->getEndEffector(eefName);

	if (!eef)
	{
		VR_ERROR << "No eef with name " << eefName << " found, aborting" << endl;
		return false;
	}

	VR_INFO << "** CREATING IK SOLVER" << endl;
	robotPlacementIK = createRobotPlacementIK(true);

	if (!robotPlacementIK )
	{
		VR_ERROR << "Could not create IK solver, aborting" << endl;
		return false;
	}
	
	robotPlacementTrajectoryIK = createRobotPlacementTrajectoryIK(true);
	if (!robotPlacementTrajectoryIK )
	{
		VR_ERROR << "Could not create trajectory IK solver, aborting" << endl;
		return false;
	}
	
	

	VR_INFO << "** FINISHED INIT" << endl;
	initialized = true;
	return true;
}


VirtualRobot::RobotPtr ReachabilityProcessor::loadRobot(const std::string &robotFile)
{
	VR_INFO << "Loading robot from " << robotFile << endl;
	RobotPtr robot;
	try
	{
		robot = RobotIO::loadRobot(robotFile);
	}
	catch (VirtualRobotException &e)
	{
		cout << " ERROR while creating robot" << endl;
		cout << e.what();
		return RobotPtr();
	}
	
	if (!robot)
	{
		cout << " ERROR while creating robot" << endl;
		return RobotPtr();
	}

	// set on floor
	BoundingBox bb = robot->getBoundingBox();
	float h = -bb.getMin()(2);
	Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
	gp(2,3) = h;
	robot->setGlobalPose(gp);
	return robot;
}

ManipulabilityPtr ReachabilityProcessor::loadReachFile(const std::string &filename)
{
	if (!robot)
		return VirtualRobot::ManipulabilityPtr();
	RuntimeEnvironment::getDataFileAbsolute(reachFile);

	ManipulabilityPtr reachSpace(new Manipulability(robot));
	try 
	{
		reachSpace->load(reachFile);
	} catch (VirtualRobotException &e)
	{
		cout << " ERROR while loading reachability space" << endl;
		cout << e.what();
		return VirtualRobot::ManipulabilityPtr();
	}
	return reachSpace;
}


RobotPlacementIKPtr ReachabilityProcessor::createRobotPlacementIK(bool lazyGridUpdate)
{
	if (!reachSpace)
		return RobotPlacementIKPtr();
	clock_t startTime = clock();
	RobotPlacementIKPtr iKSolver(new RobotPlacementIK(reachSpace->getBaseNode(),reachSpace->getNodeSet(),invReach,lazyGridUpdate));
	iKSolver->setupJacobian(0.8f,20);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	VR_INFO << "Time for building ik solver (ms):" << timeMS << endl;
	return iKSolver;
}

RobotPlacementTrajectoryIKPtr ReachabilityProcessor::createRobotPlacementTrajectoryIK(bool lazyGridUpdate)
{
	if (!invReach)
		return RobotPlacementTrajectoryIKPtr();
	clock_t startTime = clock();
	RobotPlacementTrajectoryIKPtr ik(new RobotPlacementTrajectoryIK(reachSpace->getBaseNode(),reachSpace->getNodeSet(),invReach,lazyGridUpdate));
	ik->setupJacobian(0.8f,20);
	clock_t startTime2 = clock();
	int timeMS = (int)(startTime2-startTime);
	cout << "Time for building trajectory ik solver (ms):" << timeMS << endl;
	return ik;
}


ReachabilityProcessor::IKResult ReachabilityProcessor::ik(Eigen::Matrix4f &target)
{
	return doIK(target,true,false);
}

std::vector<ReachabilityProcessor::IKResult> ReachabilityProcessor::ikTrajectory(std::vector<Eigen::Matrix4f> &target)
{
    return doIKTrajectory(target,true,false);
}

ReachabilityProcessor::IKResult ReachabilityProcessor::doIK(Eigen::Matrix4f &targetPose, bool output, bool considerCollision, int minQuality)
{
	ReachabilityProcessor::IKResult result;
    if (!initialized || !robotPlacementIK)
        return result;
	if (currentRobotNodeSet)
	{
		std::vector<float> jv(currentRobotNodeSet->getSize(),0.0f);
		currentRobotNodeSet->setJointValues(jv);
	}

	// setup collision detection
	if (considerCollision && environment && robot && robot->hasRobotNodeSet(rnsNameCollisionDetection))
	{
		if (output)
		{
			cout << "Setting up collision detection:" << endl;
			cout << "\t" << environment->getName() << "<->" << rnsNameCollisionDetection << endl;
			cout << "\t" << environment->getName() << "<->" << currentRobotNodeColSet->getName() << endl;
		}
		VirtualRobot::CDManagerPtr cdm(new CDManager());
		RobotNodeSetPtr rnsCol = robot->getRobotNodeSet(rnsNameCollisionDetection);
		cdm->addCollisionModelPair(environment,rnsCol);
		cdm->addCollisionModelPair(environment,currentRobotNodeColSet);
		robotPlacementIK->collisionDetection(cdm);
	} else
	{
		// disable collision detection
		robotPlacementIK->collisionDetection(VirtualRobot::CDManagerPtr());
	}

	robotPlacementIK->setVerbose(output);

	clock_t startTime,endTime;
	bool res;

	startTime = clock();
	res = robotPlacementIK->solve(targetPose,VirtualRobot::IKSolver::All,500,minQuality);
	endTime = clock();

	int timeMS = (int)(endTime-startTime);
	if (output)
	{
		VR_INFO << "Time for IK including search for a potential robot base pose (ms):" << timeMS << endl;
		robotPlacementIK->getReachGrid()->print();
		robotPlacementIK->printPerformanceMeasure();
	}
	result.target = targetPose;
	result.valid = res;
	if (res)
	{
		result.poseRobot = robot->getGlobalPose(); // todo: check for base node?
		if (footL)
			result.poseFootLeft = footL->getGlobalPose();
		if (footR)
			result.poseFootLeft = footR->getGlobalPose();
		result.robotConfig = robot->getConfig();
	}
	return result;
}


std::vector<ReachabilityProcessor::IKResult> ReachabilityProcessor::doIKTrajectory(std::vector<Eigen::Matrix4f> &trajectory, bool output, bool considerCollision, int minQuality)
{
	std::vector<ReachabilityProcessor::IKResult> result;
    if (!initialized || !robotPlacementTrajectoryIK)
        return result;
	if (currentRobotNodeSet)
	{
		std::vector<float> jv(currentRobotNodeSet->getSize(),0.0f);
		currentRobotNodeSet->setJointValues(jv);
	}
	
	// setup collision detection
	if (considerCollision && environment && robot && robot->hasRobotNodeSet(rnsNameCollisionDetection))
	{
		if (output)
		{
			cout << "Setting up collision detection:" << endl;
			cout << "\t" << environment->getName() << "<->" << rnsNameCollisionDetection << endl;
			cout << "\t" << environment->getName() << "<->" << currentRobotNodeColSet->getName() << endl;
		}
		VirtualRobot::CDManagerPtr cdm(new CDManager());
		RobotNodeSetPtr rnsCol = robot->getRobotNodeSet(rnsNameCollisionDetection);
		cdm->addCollisionModelPair(environment,rnsCol);
		cdm->addCollisionModelPair(environment,currentRobotNodeColSet);
		robotPlacementTrajectoryIK->collisionDetection(cdm);
	} else
	{
		// disable collision detection
		robotPlacementTrajectoryIK->collisionDetection(VirtualRobot::CDManagerPtr());
	}

	robotPlacementTrajectoryIK->setVerbose(output);

	clock_t startTime,endTime;

	startTime = clock();
	RobotPlacementTrajectoryIK::TrajectoryIK trajectorySolution = robotPlacementTrajectoryIK->solve(trajectory,VirtualRobot::IKSolver::All,500);
	endTime = clock();

	int timeMS = (int)(endTime-startTime);
	if (output)
	{
		VR_INFO << "Time for trajectory IK (ms):" << timeMS << endl;
		robotPlacementTrajectoryIK->getReachGrid()->print();
		robotPlacementTrajectoryIK->printPerformanceMeasure();
	}
	if (trajectorySolution.valid)
	{
	    VR_ASSERT (trajectorySolution.wsTrajectory.size() == trajectorySolution.jsTrajectory.size());
	    
	    for (size_t i=0;i<trajectorySolution.wsTrajectory.size();i++)
	    {
	        IKResult r;
	        r.valid = true;
	        r.target = trajectorySolution.wsTrajectory[i];
	        r.poseRobot = trajectorySolution.robotPose;
	        reachSpace->getNodeSet()->setJointValues(trajectorySolution.jsTrajectory[i].jointValues);
	        
    		if (footL)
    			r.poseFootLeft = footL->getGlobalPose();
    		if (footR)
    			r.poseFootLeft = footR->getGlobalPose();
    		r.robotConfig = robot->getConfig();
    		result.push_back(r);
    	}
	}
	return result;
}


float ReachabilityProcessor::getBaseHeight()
{
	if (reachSpace && reachSpace->getBaseNode())
		return reachSpace->getBaseNode()->getGlobalPose()(2,3);
	return 0;
}

}
