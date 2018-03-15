#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include "showCamWindow.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "CameraViewer");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("cam1");
    VirtualRobot::RuntimeEnvironment::considerKey("cam2");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    //std::string cam1Name("DepthCameraSim");
    std::string cam1Name("EyeRightCamera");
    std::string cam2Name("EyeLeftCamera");

    std::string filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");

    if (VirtualRobot::RuntimeEnvironment::hasValue("cam1"))
    {
        cam1Name = VirtualRobot::RuntimeEnvironment::getValue("cam1");
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("cam2"))
    {
        cam2Name = VirtualRobot::RuntimeEnvironment::getValue("cam2");
    }

    VR_INFO << "Robot file:" << filename << ", cam1:" << cam1Name << ", cam2:" << cam2Name << endl;
    showCamWindow rw(filename, cam1Name, cam2Name);

    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
