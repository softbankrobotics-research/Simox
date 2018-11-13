#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>


#include "exceptions.h"
#include "SimoxXMLDocument.h"
#include "MjcfDocument.h"


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
        
        void convertToMjcf();
        
        void setPaths(const std::string& inputFilename, 
                      const std::string& outputDirectory);
        
        void makeEnvironment();
        
        void addNodeBodies();
        mjcf::Element* addNodeBody(RobotNodePtr node);
        
        void addNodeBodyMeshes();


        void sanitizeMasslessBodies();
        void sanitizeMasslessBodyRecursion(mjcf::Element* body);
        
        void sanitizeMasslessLeafBody(mjcf::Element* body);
        
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
        
        std::map<std::string, mjcf::Element*> nodeBodies;

        
    };

}
