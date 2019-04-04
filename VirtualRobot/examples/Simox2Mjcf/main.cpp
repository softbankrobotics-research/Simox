#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/mujoco/MujocoIO.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = boost::filesystem;


void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 
              << " --robot <simox robot file>"
              << " [--outdir <output directory>]"
              << " [--meshRelDir <relative mesh directory>]"
              << " [--actuator {motor|position|velocity}]"
              << " [--mocap {y|n}]"
              << " [--scale_length <length scaling (to m)>]"
              << " [--scale_mesh <mesh scaling (to m)>]"
              << " [--scale_mass <mass scaling (to kg)>]"
              << " [--verbose {y|n}]"
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
    RuntimeEnvironment::considerKey("scale_length");
    RuntimeEnvironment::considerKey("scale_mesh");
    RuntimeEnvironment::considerKey("scale_mass");
    RuntimeEnvironment::considerKey("verbose");
    
    RuntimeEnvironment::processCommandLine(argc, argv);

    RuntimeEnvironment::print();
    
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
    
    const fs::path outputDir = RuntimeEnvironment::checkParameter(
                "outdir", (inputFilename.parent_path() / "mjcf").string());
    const std::string meshRelDir = RuntimeEnvironment::checkParameter("meshRelDir", "mesh");
    
    
    const std::string actuatorTypeStr = RuntimeEnvironment::checkParameter("actuator", "motor");
    mujoco::ActuatorType actuatorType;
    try
    {
        actuatorType = mujoco::toActuatorType(actuatorTypeStr);
    }
    catch (const std::out_of_range&)
    {
        std::cout << "No actuator '" << actuatorTypeStr << "'" << std::endl;
        std::cout << "Avaliable: motor|position|velocity" << std::endl;
        return -1;
    }
    
    const bool mocap = RuntimeEnvironment::checkParameter("mocap", "n") == "y";
    const bool verbose = RuntimeEnvironment::checkParameter("verbose", "n") == "y";
    
    auto parseFloatParameter = [](const std::string& key, const std::string& default_)
    {
        const std::string string = RuntimeEnvironment::checkParameter(key, default_);
        try
        {
            return std::stof(string);
        }
        catch (const std::invalid_argument& e)
        {
            std::cerr << "Could not parse value of argument " << key << ": '" << string << "' \n" 
                      << "Reason: " << e.what() << std::endl;
            throw e;
        }
    };
    
    float lengthScale;
    float meshScale;
    float massScale;
    try
    {
        lengthScale = parseFloatParameter("scale_length", "1");
        meshScale = parseFloatParameter("mesh_length", "1");
        massScale = parseFloatParameter("scale_mass", "1");
    }
    catch (const std::invalid_argument&)
    {
        return -1;
    }
    
    
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
    
    if (/* DISABLES CODE */ (false))
    {
        // using RobotIO
        RobotIO::saveMJCF(robot, inputFilename.filename().string(), outputDir.string(), meshRelDir);
    }
    else
    {
        // direct API
        mujoco::MujocoIO mujocoIO(robot);
        mujocoIO.setActuatorType(actuatorType);
        mujocoIO.setWithMocapBody(mocap);
        mujocoIO.setLengthScale(lengthScale);
        mujocoIO.setMeshScale(meshScale);
        mujocoIO.setMassScale(massScale);
        mujocoIO.setVerbose(verbose);
        mujocoIO.saveMJCF(inputFilename.filename().string(), outputDir.string(), meshRelDir);
    }
}
