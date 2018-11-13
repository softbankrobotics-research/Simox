#include "xml_visitors.h"
#include "utils.h"

using namespace VirtualRobot::mjcf;



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
