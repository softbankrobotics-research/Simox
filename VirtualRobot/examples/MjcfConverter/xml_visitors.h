#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot
{
namespace mjcf
{

class ContactExcludeVisitor : public tinyxml2::XMLVisitor
{
public:
    
    ContactExcludeVisitor(Document& document);
    virtual ~ContactExcludeVisitor() override {}

    // XMLVisitor interface
    virtual bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    
    
private:
    
    Document& document;  ///< The document.
    bool firstSkipped = false;  ///< Used to skip the root element.
    
};

}
} 
