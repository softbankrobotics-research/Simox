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
