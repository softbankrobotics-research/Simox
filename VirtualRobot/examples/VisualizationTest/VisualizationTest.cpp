#include "VisualizationTestWindow.h"
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Visualization Test");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    VisualizationTestWindow testWindow;
    testWindow.show();
    testWindow.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
