#include "FullDemoWindow.h"
#include <VirtualRobot/Tools/RuntimeEnvironment.h>

int main(int argc, char* argv[])
{
    QApplication qapp(argc, argv);

    VirtualRobot::init(argc, argv, "Simox Visualization Test");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    FullDemoWindow testWindow;
    testWindow.show();
    testWindow.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
