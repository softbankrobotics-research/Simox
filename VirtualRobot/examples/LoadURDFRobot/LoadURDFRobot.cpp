#include "../../Model/Model.h"
#include "../../VirtualRobotException.h"
#include "../../Model/Nodes/ModelNode.h"
#include "../../XML/ModelIO.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../Import/URDF/SimoxURDFFactory.h"
#include "../../Import/SimoxXMLFactory.h"

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    // robot file
    std::string filename("robots/Armar3/Armar3.xml");
    std::string urdfFile("robots/Armar3/urdf/Armar3.urdf");
    RuntimeEnvironment::getDataFileAbsolute(filename);
    RuntimeEnvironment::getDataFileAbsolute(urdfFile);

    // load model from file and convert it to the simox robot format
    RobotPtr r1 = ModelIO::loadRobotModel(filename);
    VR_ASSERT(r1);

    VR_INFO << "Loaded simox model (with urdf description): " << r1->getName() << endl;

    SimoxURDFFactory f;
    RobotPtr r2 = f.loadFromFile(urdfFile);
    VR_ASSERT(r2);

    VR_INFO << "Loaded URDF model directly (without simox attachments): " << r2->getName() << endl;

    //std::string outPath = boost::filesystem::initial_path().generic_string();
    //cout << "Saving converted file to " << outPath << "/urdf_output.xml..." << endl;

    //SimoxXMLFactory::saveXML(r, "urdf_output.xml", outPath);
}
