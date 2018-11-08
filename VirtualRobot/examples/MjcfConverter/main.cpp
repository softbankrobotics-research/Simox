#include <boost/filesystem.hpp>

#include <VirtualRobot/RuntimeEnvironment.h>

#include "MjcfConverter.h"


using namespace VirtualRobot;


#define VR_INFO    std::cout << "[INFO] "
#define VR_WARNING std::cerr << "[WARN] "
#define VR_ERROR   std::cerr << "[ERROR] "


int main(int argc, char* argv[])
{
    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
//    VirtualRobot::RuntimeEnvironment::print();

    std::string inputFilename;
    
    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            inputFilename = robFile;
        }
        else
        {
            VR_INFO << "Something is wrong with " << robFile;
        }
    }
    else
    {
        VR_INFO << "Usage: " << argv[0] << " --robot <simox robot file>";
        return 0;
    }
    
    boost::filesystem::path outputDir(inputFilename);
    outputDir.remove_filename();
    outputDir /= "mjcf";
    
    VR_INFO << "Input file:  " << inputFilename << std::endl;
    VR_INFO << "Output dir: " << outputDir << std::endl;

    MjcfConverter converter;
    converter.convert(inputFilename, outputDir.string());
    
}
