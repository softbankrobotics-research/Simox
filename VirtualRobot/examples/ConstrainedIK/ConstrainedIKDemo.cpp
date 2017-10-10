/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Peter Kaiser
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/Model/Nodes/ModelNode.h"
#include "VirtualRobot/XML/ModelIO.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include "VirtualRobot/Tools/RuntimeEnvironment.h"

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ConstrainedIKWindow.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Constrained IK Demo");
    std::cout << " --- START --- " << std::endl;

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename = VirtualRobot::RuntimeEnvironment::checkValidFileParameter("robot", "robots/Armar3/Armar3.xml");

    std::cout << "Using robot file " << filename << std::endl;

    ConstrainedIKWindow rw(filename);

    rw.main();

    return 0;
}
