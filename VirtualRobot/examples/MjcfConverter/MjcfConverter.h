#pragma once

#include <boost/filesystem.hpp>

#include <VirtualRobot/Robot.h>


#include "exceptions.h"
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
        
        void gatherCollisionAndVisualizationFiles();
        void addNodeBodies();
        void addNodeBodyMeshes();
        
        mjcf::Element* addNodeBody(RobotNodePtr node);


        void sanitizeMasslessBodies();
        void sanitizeMasslessBodyRecursion(mjcf::Element* body);
        
        void sanitizeMasslessLeafBody(mjcf::Element* body);
        
        
        

        // Paths
        
        boost::filesystem::path inputFilePath;
        boost::filesystem::path inputFileName;
        boost::filesystem::path inputFileDirectory;
        
        boost::filesystem::path outputDirectory;
        boost::filesystem::path outputFileName;
        
        boost::filesystem::path outputMeshRelDirectory;

        
        // Input
        
        std::unique_ptr<tinyxml2::XMLDocument> inputXML;
        RobotPtr robot;
        
        
        // Output
        
        mjcf::DocumentPtr document = nullptr;
        
        
        // Processing
        
        std::map<std::string, mjcf::Element*> nodeBodies;
        
        std::map<std::string, std::string> nodeCollisionModelFiles;
        std::map<std::string, std::string> nodeVisualizationFiles;
        
    };

}
