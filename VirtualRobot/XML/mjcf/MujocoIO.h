#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>


#include "MjcfDocument.h"
#include "MasslessBodySanitizer.h"


namespace VirtualRobot
{
namespace mjcf
{


    class MujocoIO
    {
    public:
        
        MujocoIO();
        
        
        void saveMJCF(RobotPtr robot, const std::string& filename, 
                      const std::string& basePath, const std::string& meshDir);
        
        
    private:
        
        
        void setPaths(const std::string& inputFilename, 
                      const std::string& outputDirectory);
        
        void makeEnvironment();
        
        void addNodeBodies();
        mjcf::Element* addNodeBody(RobotNodePtr node);
        
        void addNodeBodyMeshes();


        void addContactExcludes();
        
        void addActuators();
        
        void scaleLengths();
        
        std::vector<const mjcf::Element*> getAllElements(const std::string& elemName);

        
        // Paremeters
        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        
        
        // Paths
        
        boost::filesystem::path outputDirectory;
        boost::filesystem::path outputFileName;
        boost::filesystem::path outputMeshRelDirectory;

        
        // Input
        
        RobotPtr robot;
        
        
        // Output
        
        mjcf::DocumentPtr document = nullptr;
        
        
        // Processing
        
        mjcf::MasslessBodySanitizer masslessBodySanitizer;
        
        std::map<std::string, mjcf::Element*> nodeBodies;
        
        std::map<std::string, std::string> mergedBodyNames;

    };
    
}
}
