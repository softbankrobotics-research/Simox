#pragma once

#include <VirtualRobot/Robot.h>

#include "MjcfDocument.h"


namespace VirtualRobot
{

    class MjcfConverter
    {
    public:
        
        MjcfConverter();
        
        
        void convert(const std::string& inputSimoxXmlFile,
                     const std::string& outputMjcfFile);
        
        
    private:
        
        void loadInputFile(const std::string& inputFilename);
        void writeOutputFile(const std::string& outputFilename);
        
        void convertToMjcf();
        
        mjcf::Element* addNodeBody(RobotNodePtr node);
        
        
        RobotPtr robot;
        mjcf::DocumentPtr document = nullptr;
        
        std::map<std::string, mjcf::Element*> addedBodys;
        
    };

}
