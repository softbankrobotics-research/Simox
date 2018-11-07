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
        
        RobotPtr loadInputFile(const std::string& inputFilename);
        
        MjcfDocumentPtr convertToMjcf(RobotPtr robot);
        
        void writeOutputFile(MjcfDocument& mjcfDoc,
                             const std::string& outputFilename);
        
    };

}
