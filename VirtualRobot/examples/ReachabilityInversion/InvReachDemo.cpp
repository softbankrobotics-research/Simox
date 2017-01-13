#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "InvReachWindow.h"

#define MANIPULABILITY
//#define ATLAS

using std::cout;
using std::endl;
using namespace VirtualRobot;

int main(int argc, char *argv[])
{
	SoDB::init();
	SoQt::init(argc,argv,"ReachabilityMap");
	cout << " --- START --- " << endl;

#ifdef ATLAS
	std::string eef("Hand R");
    // RIGHT ARM
    std::string filenameReach("/common/homes/staff/vahrenka/coding/simox_projects_redmine/DRC/Reachability/ATLAS_manip_HipRightArm4.bin");
    std::string invReachFile("/common/homes/staff/vahrenka/coding/simox_projects_redmine/DRC/Reachability/ATLAS_invmanip_HipRightArm.bin");

	// LEFT ARM
	//std::string filenameReach(DEMO_BASE_DIR "/../../DRC/Reachability/ATLAS_manip_HipLeftArm2.bin");
	//std::string invReachFile(DEMO_BASE_DIR "/../../DRC/Reachability/ATLAS_invmanip_HipLeftArm.bin");

    std::string filenameRob("/common/homes/staff/vahrenka/coding/simox_projects_redmine/DRC/Model/atlas_v3_convex.xml");
    std::string rnsNameCollisionDetection("TorsoLegsColModel");
    std::string rnFootL("l_foot");
    std::string rnFoorR("r_foot");

#else
    // ARMAR-III
    std::string eef("Hand R");
    std::string filenameReach("/common/homes/staff/vahrenka/coding/Simox_Projects/OrientedReachabilityMap/data/Manipulability_Armar_TorsoRightArm_EulerXYZ_3.bin");
    std::string invReachFile("/common/homes/staff/vahrenka/coding/Simox_Projects/OrientedReachabilityMap/data/Manipulability_Armar_TorsoRightArm_EulerXYZ_3_inv.bin");

    std::string filenameRob("robots/ArmarIII/ArmarIII.xml");
    std::string rnsNameCollisionDetection("PlatformTorsoHeadColModel");
    std::string rnFootL("Platform");
    std::string rnFoorR;
#endif
	if (invReachFile.empty())
	{
		VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameReach);
		invReachFile = filenameReach + std::string(".inv.bin");
	}

    std::vector< std::string > envFiles;
    envFiles.push_back("objects/Table.xml");
    envFiles.push_back("/common/homes/staff/vahrenka/coding/Simox_Projects/OrientedReachabilityMap/data/Kitchen_OpenDoor.xml");

	VirtualRobot::RuntimeEnvironment::considerKey("robot");
	VirtualRobot::RuntimeEnvironment::considerKey("eef");
	VirtualRobot::RuntimeEnvironment::considerKey("reachability");
	VirtualRobot::RuntimeEnvironment::considerKey("object");
	VirtualRobot::RuntimeEnvironment::considerKey("lfoot");
	VirtualRobot::RuntimeEnvironment::considerKey("rfoot");
	VirtualRobot::RuntimeEnvironment::processCommandLine(argc,argv);
	VirtualRobot::RuntimeEnvironment::print();

	cout << " --- START --- " << endl;

	if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
	{
		std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
		if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
		{
			filenameRob = robFile;
		}
	}
	if (VirtualRobot::RuntimeEnvironment::hasValue("reachability"))
	{
		std::string reachFile = VirtualRobot::RuntimeEnvironment::getValue("reachability");
		if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFile))
		{
			filenameReach = reachFile;
		}
	}

	if (VirtualRobot::RuntimeEnvironment::hasValue("eef"))
		eef = VirtualRobot::RuntimeEnvironment::getValue("eef");
	if (VirtualRobot::RuntimeEnvironment::hasValue("lfoot"))
		rnFootL = VirtualRobot::RuntimeEnvironment::getValue("lfoot");
	if (VirtualRobot::RuntimeEnvironment::hasValue("rfoot"))
		rnFoorR = VirtualRobot::RuntimeEnvironment::getValue("rfoot");

	cout << "Using robot at " << filenameRob << endl;
	cout << "Using eef " << eef << endl;
	cout << "Using Feet L: " << rnFootL << ", R: " << rnFoorR << endl;
	cout << "Using eef " << eef << endl;
	cout << "Using reachability file from " << filenameReach << endl;
	cout << "Using inverse reachability file (will be created if missing): " << invReachFile << endl;

    InvReachWindow rw(filenameRob,filenameReach,eef,invReachFile,rnsNameCollisionDetection, rnFootL, rnFoorR, envFiles);

	rw.main();

	return 0;

}
