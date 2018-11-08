#include "xml_visitors.h"

using namespace VirtualRobot::mjcf;


ContactExcludeVisitor::ContactExcludeVisitor(Document& document) : 
    document(document)
{
}

bool ContactExcludeVisitor::VisitEnter(
        const tinyxml2::XMLElement& body, const tinyxml2::XMLAttribute*)
{
    if (std::string(body.Value()) != "body")
    {
        return true;
    }
    
    if (!firstSkipped)
    {
        firstSkipped = true;
        return true;
    }
    
    const Element* parent = body.Parent()->ToElement();
    assert(parent);
    
    document.addContactExclude(*parent, body);
    
    return true;
}

