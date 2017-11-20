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

#include "GenericIKWindow.h"

bool useColModel = false;


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Generic IK Demo");
    cout << " --- START --- " << endl;


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename("robots/Armar3/Armar3.xml");
    //std::string filename("robots/iCub/iCub.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    cout << "Using robot at " << filename << endl;

    GenericIKWindow rw(filename);

    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
