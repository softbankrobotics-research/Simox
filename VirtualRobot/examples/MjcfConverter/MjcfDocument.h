#pragma once

#include <memory>
#include <tinyxml2.h>


namespace VirtualRobot
{

    class MjcfDocument : public tinyxml2::XMLDocument
    {
        
    public:
        
        MjcfDocument();
        
        
    };
    
    using MjcfDocumentPtr = std::unique_ptr<MjcfDocument>;
    
}
