
#include "MTPlanningWindow.h"

#include <string.h>
#include <iostream>

using namespace std;
using namespace VirtualRobot;

void startMTPlanning(const std::string &robotFile)
{
    MTPlanningWindow* agfw = new MTPlanningWindow(robotFile);
    agfw->main();
    delete agfw;
}

int main(int argc, char** argv)
{
    VirtualRobot::init(argc, argv, "Multi-Threaded-Planning-Demo");
    cout << " --- START --- " << endl;

    std::string robotFile = "robots/SimoxXML/examples/MultiThreadedPlanning/CartMover.xml";

    if (!CollisionChecker::IsSupported_Multithreading_MultipleColCheckers())
    {
        cout << " The collision detection library that is linked to simox does not support multi threading. Aborting...." << endl;
        return -1;
    }

    try
    {
        startMTPlanning(robotFile);
    }
    catch (std::exception e)
    {
        std::cout << "Exception: " << e.what() << std::endl ;
    }
    catch (...)
    {
        ;
    }

    cout << " --- END --- " << endl;

    return 0;
}
