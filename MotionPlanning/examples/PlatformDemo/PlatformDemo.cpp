
#include "PlatformWindow.h"

#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>


int main(int argc, char** argv)
{
    VirtualRobot::init(argc, argv, "GraspRrtDemo");
 
    cout << " --- START --- " << endl;

    std::string filenameScene("/scenes/examples/PlatformDemo/PlatformDemo.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameScene);
    std::string rnsName("Planning");
    std::string colModel1("ColModel Robot Body");
    std::string colModel2("ColModel Obstacles");


    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::considerKey("robotNodeSet");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelEnv");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string scFile = VirtualRobot::RuntimeEnvironment::getValue("scene");

    if (!scFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(scFile))
    {
        filenameScene = scFile;
    }

    std::string rns = VirtualRobot::RuntimeEnvironment::getValue("robotNodeSet_A");

    if (!rns.empty())
    {
        rnsName = rns;
    }

    std::string c1 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot");

    if (!c1.empty())
    {
        colModel1 = c1;
    }

    std::string c3 = VirtualRobot::RuntimeEnvironment::getValue("colModelEnv");

    if (!c3.empty())
    {
        colModel2 = c3;
    }


    cout << "Using scene: " << filenameScene << endl;
    cout << "Using environment collision model set: <" << colModel2 << ">" << endl;
    cout << "\t Using RobotNodeSet for planning: <" << rnsName << ">" << endl;
    cout << "\t Using robot collision model set: <" << colModel1 << ">" << endl;

    PlatformWindow rw(filenameScene, rnsName, colModel1, colModel2);

    rw.main();

    return 0;
}
