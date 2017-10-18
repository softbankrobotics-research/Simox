#include "../../../VirtualRobot/Model/Model.h"
#include "../../../VirtualRobot/VirtualRobotException.h"
#include "../../../VirtualRobot/Model/Nodes/ModelNode.h"
#include "../../../VirtualRobot/XML/ModelIO.h"
#include "../../../VirtualRobot/Visualization/VisualizationFactory.h"
#include "../../../VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "../../../VirtualRobot/Tools/RuntimeEnvironment.h"

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "showSceneWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Scene Viewer");

    cout << " --- START --- " << endl;
    std::string filename("scenes/examples/SceneViewer/scene1.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("scene", filename);
    showSceneWindow rw(filename);

    rw.main();

    return 0;
}
