#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelJoint.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Tools/RuntimeEnvironment.h"


#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename("robots/Armar3/Armar3.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    VR_INFO << "Robot file: " << filename << endl;
    RobotPtr rob;

    try
    {
        rob = ModelIO::loadRobotModel(filename, ModelIO::eStructure);
    }
    catch (VirtualRobotException& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    if (rob)
    {
        rob->print();
    }
    else
    {
        VR_INFO << " ERROR while creating robot" << endl;
    }
}
