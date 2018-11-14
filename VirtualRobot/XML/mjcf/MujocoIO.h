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
        mjcf::XMLElement* addNodeBody(RobotNodePtr node);
        
        void addNodeBodyMeshes();


        void addContactExcludes();
        
        void addActuators();
        
        void scaleLengths();
        
        std::vector<const mjcf::XMLElement*> getAllElements(const std::string& elemName);

        
        // Paremeters
        
        /// Scaling for lengths, such as positions and translations.
        float lengthScaling = 0.001f;
        
        
        // Paths
        
        boost::filesystem::path outputDirectory;
        boost::filesystem::path outputFileName;
        boost::filesystem::path outputMeshRelDirectory;

        
        // Input
        
        /// The input robot.
        RobotPtr robot;
        
        
        // Output
        
        /// The built MJCF document.
        DocumentPtr document = nullptr;
        
        
        // Processing
        
        /// Sanitizes massless bodies.
        mjcf::MasslessBodySanitizer masslessBodySanitizer;
        
        /// Map of robot node names to XML elements.
        std::map<std::string, mjcf::XMLElement*> nodeBodies;
        
    };
    
}
}
