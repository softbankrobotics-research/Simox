#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Tools/RuntimeEnvironment.h"

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "IKRRTWindow.h"

//#define ARMAR


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "IK-RRT Demo");
    cout << " --- START --- " << endl;

#ifdef ARMAR
    std::string filenameScene("scenes/examples/IKRRT/planningHotSpot.xml");
    std::string filenameReach("reachability/ArmarIII_HipLeftArm.bin");
    std::string kinChain("TorsoLeftArm");
    std::string eef("Hand L");
    std::string colModel("LeftArmHandColModel");
    std::string colModelRob("PlatformTorsoHeadColModel");
#else
    // ICUB
    std::string filenameScene("scenes/examples/IKRRT/IKRRT_scene_iCub.xml");
    std::string filenameReach("reachability/iCub_HipLeftArm.bin");
    std::string kinChain("Hip Left Arm");
    std::string eef("Left Hand");
    std::string colModel("Left HandArm ColModel");
    std::string colModelRob("BodyHeadLegsColModel");
#endif
    if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameScene))
    {
        VR_ERROR << "Could not get absolute path of file: " << filenameScene << std::endl;
    }
    if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameReach))
    {
        VR_ERROR << "Could not get absolute path of file: " << filenameReach << std::endl;
    }

    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::considerKey("reachability");
    VirtualRobot::RuntimeEnvironment::considerKey("kinematicChain");
    VirtualRobot::RuntimeEnvironment::considerKey("endEffector");
    VirtualRobot::RuntimeEnvironment::considerKey("collisionModelKinChain");
    VirtualRobot::RuntimeEnvironment::considerKey("collisionModelRobot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string scFile = VirtualRobot::RuntimeEnvironment::getValue("scene");

    if (!scFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(scFile))
    {
        filenameScene = scFile;
    }

    std::string reachFile = VirtualRobot::RuntimeEnvironment::getValue("reachability");

    if (!reachFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFile))
    {
        filenameReach = reachFile;
    }

    std::string kc = VirtualRobot::RuntimeEnvironment::getValue("kinematicChain");

    if (!kc.empty())
    {
        kinChain = kc;
    }

    std::string EndEff = VirtualRobot::RuntimeEnvironment::getValue("endEffector");

    if (!EndEff.empty())
    {
        eef = EndEff;
    }

    std::string col1 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelKinChain");

    if (!col1.empty())
    {
        colModel = col1;
    }

    std::string col2 = VirtualRobot::RuntimeEnvironment::getValue("collisionModelRobot");

    if (!col2.empty())
    {
        colModelRob = col2;
    }


    cout << "Using scene at " << filenameScene << endl;
    cout << "Using reachability at " << filenameReach << endl;
    cout << "Using end effector " << eef << endl;
    cout << "Using col model (kin chain) " << colModel << endl;
    cout << "Using col model (static robot)" << colModelRob << endl;

    IKRRTWindow rw(filenameScene, filenameReach, kinChain, eef, colModel, colModelRob);

    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
