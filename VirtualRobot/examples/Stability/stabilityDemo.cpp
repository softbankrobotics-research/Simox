#include "stabilityWindow.h"
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

using namespace VirtualRobot;

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Stability Demo"); 
    cout << " --- START --- " << endl;

    std::string filenameRob("robots/SimoxXML/iCub/iCub.xml");
    std::string linkset("Hip Left Arm Masses");
    std::string jointset("Hip Left Arm");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameRob);

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("linkset");
    VirtualRobot::RuntimeEnvironment::considerKey("jointset");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    cout << " --- START --- " << endl;

    if (VirtualRobot::RuntimeEnvironment::hasValue("linkset"))
        linkset = VirtualRobot::RuntimeEnvironment::getValue("linkset");

    if (VirtualRobot::RuntimeEnvironment::hasValue("jointset"))
        jointset = VirtualRobot::RuntimeEnvironment::getValue("jointset");

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filenameRob = robFile;
        }
    }

    cout << "Using robot file " << filenameRob << ", link set:" << linkset << ", joint set: " << jointset << endl;

    stabilityWindow rw(filenameRob, linkset, jointset);

    rw.show();
    rw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
