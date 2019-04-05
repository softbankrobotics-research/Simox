#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/mujoco/MujocoIO.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = boost::filesystem;


/**
 * Loads a Simox robot and converts it to Mujoco's XML format (MJCF).
 * The converted file is stored in a directory mjcf/ placed in the directory
 * of the input file.
 */
int main(int argc, char* argv[])
{
    RuntimeEnvironment::setCaption("Convert Simox XML to MJCF (Mujoco XML)");
    
    RuntimeEnvironment::considerKey(
                "robot", "The Simox robot model to convert. (required)");
    RuntimeEnvironment::considerKey(
                "outdir", "The output directory. (default: 'mjcf')");
    RuntimeEnvironment::considerKey(
                "meshRelDir", "The mesh directory relative to outdir. (default: 'mesh')");
    RuntimeEnvironment::considerKey(
                "actuator", "The actuator type to add (motor, position, velocity). (default: motor)");
    RuntimeEnvironment::considerFlag(
                "mocap", "Add a mocap body to which the robot root body is welded.");
    
    RuntimeEnvironment::considerKey(
                "scale_length", "Scaling of lengths and distances (to m). For meshes, see 'scale_mesh'. (default: 1.0)");
    RuntimeEnvironment::considerKey(
                "scale_mesh", "Scaling of meshes (to m). (default: 1.0)");
    RuntimeEnvironment::considerKey(
                "scale_mass", "Scaling of masses (to kg). (default: 1.0)");
    
    RuntimeEnvironment::considerFlag(
                "actuator_suffix", "Add a type suffix to actuator names.");
    
    RuntimeEnvironment::considerFlag(
                "verbose", "Enable verbose output.");
    
    RuntimeEnvironment::processCommandLine(argc, argv);

    
    if (RuntimeEnvironment::hasHelpFlag() || !RuntimeEnvironment::hasValue("robot"))
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }
    
    fs::path inputFilename;
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
        std::cout << "No actuator type '" << actuatorTypeStr << "'" << std::endl;
        std::cout << "Avaliable: motor|position|velocity" << std::endl;
        return -1;
    }
    
    const bool mocap = RuntimeEnvironment::hasFlag("mocap");
    const bool verbose = RuntimeEnvironment::hasFlag("verbose");
    const bool addActuatorSuffix = RuntimeEnvironment::hasFlag("actuator_suffix");
    
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
    
    float scaleLength;
    float scaleMesh;
    float scaleMass;
    try
    {
        scaleLength = parseFloatParameter("scale_length", "1");
        scaleMesh = parseFloatParameter("mesh_length", "1");
        scaleMass = parseFloatParameter("scale_mass", "1");
    }
    catch (const std::invalid_argument&)
    {
        return -1;
    }
    
    
    std::cout << "Input file:      " << inputFilename << std::endl;
    std::cout << "Output dir:      " << outputDir << std::endl;
    std::cout << "Output mesh dir: " << outputDir / meshRelDir << std::endl;
    std::cout << "Actuator type:   " << actuatorTypeStr << std::endl;
    std::cout << "Mocap body:      " << (mocap ? "yes" : "no ") << std::endl;
    
    std::cout << "Scaling: " <<  std::endl
              << "  - length: " << scaleLength << std::endl
              << "  - mesh:   " << scaleMesh << std::endl
              << "  - mass:   " << scaleMass << std::endl;

    std::cout << "Loading robot ..." << std::endl;
    
    RobotPtr robot;
    try
    {
        std::cout << "Loading robot from " << inputFilename << " ..." << std::endl;
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
        mujocoIO.setLengthScale(scaleLength);
        mujocoIO.setMeshScale(scaleMesh);
        mujocoIO.setMassScale(scaleMass);
        mujocoIO.setVerbose(verbose);
        mujocoIO.setAddActuatorTypeSuffix(addActuatorSuffix);
        
        mujocoIO.saveMJCF(inputFilename.filename().string(), outputDir.string(), meshRelDir);
    }
}
