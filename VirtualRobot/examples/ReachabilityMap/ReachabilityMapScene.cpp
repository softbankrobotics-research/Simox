#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/RuntimeEnvironment.h"

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ReachabilityMapWindow.h"

//#define ICUB

using std::cout;
using std::endl;
using namespace VirtualRobot;

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Reachability Map Demo");

    cout << " --- START --- " << endl;

    std::string filenameReach;
    std::string eef;
#ifdef ICUB
    std::string filenameRob("robots/iCub/iCub.xml");
    std::string fileObj("objects/iCub/LegoXWing_RightHand_300.xml");
    filenameReach = "reachability/iCub_HipLeftArm.bin";
#else
    std::string filenameRob("robots/Armar3/Armar3.xml");
    std::string fileObj("objects/Hammer.xml");
    filenameReach = "workspace/armar3_leftArm_reachability.bin";
    eef = "Hand L";
#endif


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("eef");
    VirtualRobot::RuntimeEnvironment::considerKey("reachability");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
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

    if (VirtualRobot::RuntimeEnvironment::hasValue("object"))
    {
        std::string of = VirtualRobot::RuntimeEnvironment::getValue("object");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(of))
        {
            fileObj = of;
        }
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("eef"))
    {
        eef = VirtualRobot::RuntimeEnvironment::getValue("eef");
    }

    cout << "Using robot at " << filenameRob << endl;
    cout << "Using eef " << eef << endl;
    cout << "Using manipulation object at " << fileObj << endl;
    cout << "Using reachability file from " << filenameReach << endl;

    ReachabilityMapWindow rw(filenameRob, filenameReach, fileObj, eef);

    rw.main();

    return 0;

}
