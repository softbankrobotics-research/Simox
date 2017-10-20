#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>
#include <VirtualRobot/Workspace/Manipulability.h>
#include <VirtualRobot/IK/PoseQualityExtendedManipulability.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "reachabilityWindow.h"

// this flag can be used to build a good representation of the workspace
// the reachability data will be extended in endless mode and
// after every 1 mio-th update a snapshot is saved.
//#define ENDLESS

//#define ICUB
//#define ARMAR3B

// --robot robots/iCub/iCub_LeftHand_Extended.xml

using std::cout;
using std::endl;
using namespace VirtualRobot;



void endlessExtend(std::string robotFile, std::string reachFile, int steps)
{
    VR_INFO << "Extending workspace information, saving each " << steps << " steps." << endl;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFile);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFile);

    // load robot
    RobotPtr robot;

    try
    {
        robot = ModelIO::loadModel(robotFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    // load reach file
    WorkspaceRepresentationPtr reachSpace;
    bool loadOK = true;

    // try manipulability file
    try
    {
        reachSpace.reset(new Manipulability(robot));
        reachSpace->load(reachFile);
    }
    catch (...)
    {
        loadOK = false;
    }

    if (!loadOK)
    {
        // try reachability file

        loadOK = true;

        try
        {
            reachSpace.reset(new Reachability(robot));
            reachSpace->load(reachFile);
        }
        catch (...)
        {
            loadOK = false;
        }
    }

    if (!loadOK)
    {
        VR_ERROR << "Could not load reach/manip file" << endl;
        reachSpace.reset();
        return;
    }


    reachSpace->print();

    time_t time_now = time(NULL);
    struct tm* timeinfo;
    timeinfo = localtime(&time_now);
    char buffer [255];
    strftime(buffer, 255, "ReachabilityData_ENDLESS_%Y_%m_%d__%H_%M_%S_", timeinfo);
    int nr = 0;

    while (true)
    {
        //reachSpace->addRandomTCPPoses(steps);
        reachSpace->addRandomTCPPoses(steps, QThread::idealThreadCount() < 1 ? 1 : QThread::idealThreadCount(), true);

        reachSpace->print();
        std::stringstream ss;
        ss << buffer << "_" << nr << ".bin";
        reachSpace->save(ss.str());
        nr++;
    }
}

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Reachability Demo");

    cout << " --- START --- " << endl;
    cout << "Hint: use '--extendReach true' to start a refinement of the reachability data. This is an endless mode which creates intermediate data files and that can be running e.g. over night." << std::endl;

    std::string filenameReach;
#if defined(ICUB)
    std::cout << "Using ICUB" << std::endl;
    std::string filenameRob("robots/iCub/iCub.xml");
    Eigen::Vector3f axisTCP(1.0f, 0, 0);
    filenameReach = "reachability/iCub_HipRightArm.bin";
#elif defined(ARMAR3B)

    std::cout << "Using ARMAR3B" << std::endl;
    std::string filenameRob("/home/SMBAD/yamanobe/home/armarx/Armar3/data/Armar3/robotmodel/ArmarIIIb.xml");
    Eigen::Vector3f axisTCP(0, 0, 1.0f);
    //filenameReach = "/home/SMBAD/yamanobe/home/armarx/Armar3/data/Armar3/reachability/ArmarIIIb_LeftArm.bin";
    //filenameReach = "/home/SMBAD/yamanobe/home/armarx/Armar3/data/Armar3/reachability/ArmarIIIb_RightArm.bin";
    filenameReach = "/home/SMBAD/yamanobe/home/armarx/simox/build_release/ReachabilityData_ENDLESS_2015_04_17__18_03_22__2024.bin";
#else
    std::cout << "Using ARMAR3" << std::endl;
    std::string filenameRob("robots/Armar3/Armar3.xml");
    Eigen::Vector3f axisTCP(0, 0, 1.0f);
    //filenameReach = "reachability/ArmarIII_PlatformHipRightArm.bin";
    filenameReach = "reachability/ArmarIII_TorsoRightArm.bin";
#endif
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameRob);


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("extendReach");
    VirtualRobot::RuntimeEnvironment::considerKey("extendReachStepsSave");
    VirtualRobot::RuntimeEnvironment::considerKey("reachability");
    VirtualRobot::RuntimeEnvironment::considerKey("visualizationTCPAxis");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    cout << " --- START --- " << endl;

    filenameRob = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", filenameRob);

    filenameReach = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("reachability", filenameReach);

    if (VirtualRobot::RuntimeEnvironment::hasValue("visualizationTCPAxis"))
    {
        std::string axisStr = VirtualRobot::RuntimeEnvironment::getValue("visualizationTCPAxis");

        if (!VirtualRobot::RuntimeEnvironment::toVector3f(axisStr, axisTCP))
        {
            cout << "Wrong axis definition:" << axisStr << endl;
        }
    }

    cout << "Using robot at " << filenameRob << endl;
    cout << "Using reachability file from " << filenameReach << endl;

    if (VirtualRobot::RuntimeEnvironment::hasValue("extendReach"))
    {
        int stepsSave = 100000;
        if (VirtualRobot::RuntimeEnvironment::hasValue("extendReachStepsSave"))
                stepsSave = VirtualRobot::RuntimeEnvironment::toInt(VirtualRobot::RuntimeEnvironment::getValue("extendReachStepsSave"));
        endlessExtend(filenameRob, filenameReach, stepsSave);
        return 0;
    }

    reachabilityWindow rw(filenameRob, filenameReach, axisTCP);
    rw.main();

    return 0;
}
