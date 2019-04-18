#include <filesystem>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/mjcf/MujocoIO.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = std::filesystem;


void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 
              << " --robot <simox robot file> "
              << "[--outdir <output directory>] "
              << "[--meshRelDir <relative mesh directory>] "
              << "[--actuator {motor|position|velocity}] "
              << "[--mocap {y|n}]"
              << std::endl;
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
    RuntimeEnvironment::considerKey("actuator");
    RuntimeEnvironment::considerKey("mocap");
    
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
    
    
    std::string actuatorTypeStr = RuntimeEnvironment::checkParameter("actuator", "motor");
    mjcf::ActuatorType actuatorType;
    try
    {
        actuatorType = mjcf::toActuatorType(actuatorTypeStr);
    }
    catch (const std::out_of_range&)
    {
        std::cout << "No actuator '" << actuatorTypeStr << "'" << std::endl;
        std::cout << "Avaliable: motor|position|velocity" << std::endl;
        return -1;
    }
    
    bool mocap = RuntimeEnvironment::checkParameter("mocap", "n") == "y";
    
    
    std::cout << "Input file:      " << inputFilename << std::endl;
    std::cout << "Output dir:      " << outputDir << std::endl;
    std::cout << "Output mesh dir: " << outputDir / meshRelDir << std::endl;
    //std::cout << "Actuator type: " << actuatorType << std::endl;
    std::cout << "With mocap body: " << (mocap ? "yes" : "no ");

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
    
    if (false)
    {
        // using RobotIO
        RobotIO::saveMJCF(robot, inputFilename.filename().string(), outputDir.string(), meshRelDir);
    }
    else
    {
        // direct API
        mjcf::MujocoIO mujocoIO(robot);
        mujocoIO.setActuatorType(actuatorType);
        mujocoIO.setWithMocapBody(mocap);
        mujocoIO.saveMJCF(inputFilename.filename().string(), outputDir.string(), meshRelDir);
    }
}
