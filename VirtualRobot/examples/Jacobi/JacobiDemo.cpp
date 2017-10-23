#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Model/Nodes/ModelNode.h>
#include <VirtualRobot/XML/ModelIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Tools/RuntimeEnvironment.h>


#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "JacobiWindow.h"

bool useColModel = false;


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Jacobi Demo");

    cout << " --- START --- " << endl;
    std::string filename("robots/Armar3/Armar3.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    JacobiWindow rw(filename);

    rw.main();

    return 0;
}
