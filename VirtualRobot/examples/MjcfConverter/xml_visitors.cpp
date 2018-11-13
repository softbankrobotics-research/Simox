#include "xml_visitors.h"
#include "utils.h"

using namespace VirtualRobot::mjcf;


ContactExcludeVisitor::ContactExcludeVisitor(Document& document) : 
    document(document)
{
}

bool ContactExcludeVisitor::VisitEnter(
        const tinyxml2::XMLElement& body, const tinyxml2::XMLAttribute*)
{
    if (!isElement(body, "body"))
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


ListElementsVisitor::ListElementsVisitor(const std::string& elemName) : 
    elementName(elemName)
{
}

bool ListElementsVisitor::VisitEnter(const tinyxml2::XMLElement& elem, const tinyxml2::XMLAttribute*)
{
    if (isElement(elem, elementName))
    {
        foundElements.push_back(&elem);
    }
    return true;
}

const std::vector<const tinyxml2::XMLElement*>& ListElementsVisitor::getFoundElements() const
{
    return foundElements;
}
