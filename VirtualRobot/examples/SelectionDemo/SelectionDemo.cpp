#include <VirtualRobot/Tools/RuntimeEnvironment.h>

#include "showDemoWindow.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Selection Demo");

    showDemoWindow dw;
    dw.show();
    dw.raise();

    VR_ASSERT(qApp);
    return qApp->exec();
}
