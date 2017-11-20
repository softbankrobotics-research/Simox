#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../../../Gui/Grasping/GraspEditor/GraspEditorWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "GraspEditor");
    VR_INFO << " --- START --- " << endl;

    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("robot");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();


    std::string filenameObj = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("object", "objects/iv/plate.xml");
    VR_INFO << "Object file: " << filenameObj << endl;

    std::string filenameRob = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");
    VR_INFO << "Robot file: " << filenameRob << endl;

    GraspEditorWindow rw(filenameObj, filenameRob);
    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
