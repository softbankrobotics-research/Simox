#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/VirtualRobotException.h>

#include <string>

#include "MjcfConverter.h"


using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
//    VirtualRobot::RuntimeEnvironment::print();

    std::string inputFilename("robots/examples/loadRobot/RobotExample.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(inputFilename);

    
    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");
        std::cout << "robFile: " << robFile << std::endl;

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            inputFilename = robFile;
        }
    }
    
    boost::filesystem::path outputDir(inputFilename);
    outputDir.remove_filename();
    outputDir /= "mjcf";
    
    std::cout << "Input file:  " << inputFilename << std::endl;
    std::cout << "Output dir: " << outputDir << std::endl;

    MjcfConverter converter;
    converter.convert(inputFilename, outputDir.string());
    
}
