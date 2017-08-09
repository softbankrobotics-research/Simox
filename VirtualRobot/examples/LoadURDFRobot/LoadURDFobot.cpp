#include "../../Model/Model.h"
#include "../../VirtualRobotException.h"
#include "../../Model/Nodes/ModelNode.h"
#include "../../XML/RobotIO.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../Import/URDF/SimoxURDFFactory.h"

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    SimoxURDFFactory f;

    // atlas file
    std::string urdfFile = ("robots/urdf/atlas_description/urdf/atlas_v3.urdf");
    RuntimeEnvironment::getDataFileAbsolute(urdfFile);

    // to ensure that 3d model files can be loaded during converting we need to add the correct data path
    boost::filesystem::path tmppath = urdfFile;
    tmppath = tmppath.parent_path();
    tmppath = tmppath / "/../..";
    std::string modelsBasePath = tmppath.generic_string();
    RuntimeEnvironment::addDataPath(modelsBasePath);

    // load model from file and convert it to the simox robot format
    RobotPtr r = f.loadFromFile(urdfFile);

    std::string outPath = boost::filesystem::initial_path().generic_string();
    cout << "Saving converted file to " << outPath << "/urdf_output.xml..." << endl;

    RobotIO::saveXML(r, "urdf_output.xml", outPath);
}
