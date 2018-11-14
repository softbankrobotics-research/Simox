#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/RobotIO.h>

using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = boost::filesystem;


void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 
              << " --robot <simox robot file> "
              << "[--outdir <output directory>] "
              << "[--meshRelDir <relative mesh directory>]"
              << std::endl;;
}


/**
 * Loads a Simox robot and converts it to Mujoco's XML format (MJCF).
 * The converted file is stored in a directory mjcf/ placed in the directory
 * of the input file.
 */
int main(int argc, char* argv[])
{
    RuntimeEnvironment::considerKey("robot");
    RuntimeEnvironment::considerKey("outdir");
    RuntimeEnvironment::considerKey("meshRelDir");
    RuntimeEnvironment::processCommandLine(argc, argv);

    //RuntimeEnvironment::print();
    
    fs::path inputFilename;
    
    
    if (RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = RuntimeEnvironment::getValue("robot");

        if (RuntimeEnvironment::getDataFileAbsolute(robFile))
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
        printUsage(argv[0]);
        return 0;
    }
    
    fs::path outputDir = RuntimeEnvironment::checkParameter(
                "outdir", (inputFilename.parent_path() / "mjcf").string());
    std::string meshRelDir = RuntimeEnvironment::checkParameter("meshRelDir", "mesh");
    
    
    std::cout << "Input file:      " << inputFilename << std::endl;
    std::cout << "Output dir:      " << outputDir << std::endl;
    std::cout << "Output mesh dir: " << outputDir / meshRelDir << std::endl;

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
    
    RobotIO::saveMJCF(robot, inputFilename.filename().string(), outputDir.string(), meshRelDir);
}
