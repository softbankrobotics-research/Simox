#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include "showRobotWindow.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Robot Viewer");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");
    VR_INFO << "Robot file: " << filename << endl;

    showRobotWindow rw(filename);
    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
