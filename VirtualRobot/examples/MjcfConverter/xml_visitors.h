#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot
{
namespace mjcf
{


class ListElementsVisitor : public tinyxml2::XMLVisitor
{
public:
    
    ListElementsVisitor(const std::string& elementName);
    virtual ~ListElementsVisitor() override {}

    // XMLVisitor interface
    virtual bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
    const std::vector<const tinyxml2::XMLElement*>& getFoundElements() const;
    
    
private:
    
    std::string elementName;
    
    std::vector<const tinyxml2::XMLElement*> foundElements;
    
};


}
} 
