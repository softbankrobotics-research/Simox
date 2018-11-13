#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/RobotIO.h>

using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);

    boost::filesystem::path inputFilename;
    
    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            inputFilename = robFile;
        }
        else
        {
            std::cout << "Something is wrong with " << robFile;
        }
    }
    else
    {
        std::cout << "Usage: " << argv[0] << " --robot <simox robot file>";
        return 0;
    }
    
    boost::filesystem::path outputDir = inputFilename;
    outputDir.remove_filename();
    outputDir /= "mjcf";
    
    std::cout << "Input file:  " << inputFilename << std::endl;
    std::cout << "Output dir: " << outputDir << std::endl;

    std::cout << "Loading robot ..." << std::endl;
    
    RobotPtr robot;
    try
    {
        robot = RobotIO::loadRobot(inputFilename.string(), RobotIO::eFull);
        assert(robot);
    }
    catch (const VirtualRobotException&)
    {
        throw; // rethrow
    }
    
    RobotIO::saveMJCF(robot, inputFilename.filename().string(), outputDir.string());
}
