#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspQualityWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Grasp Quality Demo");
    cout << " --- START --- " << endl;

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string objFile = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("object", "objects/iv/riceBox.xml");
    cout << "Object file: " << objFile << endl;

    std::string robFile = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");
    cout << "Robot file: " << robFile << endl;

    GraspQualityWindow rw(robFile, objFile);

    rw.main();

    return 0;
}
