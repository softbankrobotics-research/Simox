#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>


#include "exceptions.h"
#include "SimoxXMLDocument.h"
#include "MjcfDocument.h"
#include "MjcfMasslessBodySanitizer.h"


namespace VirtualRobot
{

    class MjcfConverter
    {
    public:
        
        MjcfConverter();
        
        
        void convert(const std::string& inputSimoxXmlFile,
                     const std::string& outputDirectory);
        
        
    private:
        
        
        void loadInputFile();
        void writeOutputFile();
        
        void setPaths(const std::string& inputFilename, 
                      const std::string& outputDirectory);
        
        void makeEnvironment();
        
        void addNodeBodies();
        mjcf::Element* addNodeBody(RobotNodePtr node);
        
        void addNodeBodyMeshes();


        void addContactExcludes();
        
        void addActuators();
        
        
        std::vector<const mjcf::Element*> getAllElements(const std::string& elemName);

        
        
        // Paths
        
        boost::filesystem::path inputFilePath;
        boost::filesystem::path inputFileName;
        boost::filesystem::path inputFileDirectory;
        
        boost::filesystem::path outputDirectory;
        boost::filesystem::path outputFileName;
        
        boost::filesystem::path outputMeshRelDirectory;

        
        // Input
        
        RobotPtr robot;
        
        
        // Output
        
        mjcf::DocumentPtr document = nullptr;
        
        
        // Processing
        
        mjcf::MjcfMasslessBodySanitizer masslessBodySanitizer;
        
        std::map<std::string, mjcf::Element*> nodeBodies;
        
        std::map<std::string, std::string> mergedBodyNames;

    };

}
