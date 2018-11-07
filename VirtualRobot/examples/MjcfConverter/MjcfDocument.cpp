#include "MjcfDocument.h"

using namespace VirtualRobot;
using namespace mjcf;

Document::Document() : root(NewElement("mujoco"))
{
    // create root element
    this->InsertEndChild(root);
}

void Document::setModelName(const std::string& name)
{
    root->SetAttribute("model", name.c_str());
}

Element* Document::addNewElement(Element* parent, const std::string& elemName)
{
    Element* elem = NewElement(elemName.c_str());
    parent->InsertEndChild(elem);
    return elem;
}

Element*Document::addBodyElement(Element* parent, const std::string& bodyName, 
                                 const Eigen::Vector3f& pos, const Eigen::Quaternionf& quat)
{
    Element* element = addNewElement(parent, "body");
    
    element->SetAttribute("name", bodyName.c_str());
    
    if (!pos.isZero(floatCompPrecision))
    {
        element->SetAttribute("pos", toAttrVal(pos).c_str());
    }
    if (!quat.isApprox(Eigen::Quaternionf::Identity(), floatCompPrecision))
    {
        element->SetAttribute("quat", toAttrVal(quat).c_str());
    }
    
    return element;
}

std::string Document::toAttrVal(const Eigen::Vector3f& v)
{
    std::stringstream ss;
    ss << v.format(iofVector);
    return ss.str();
}

std::string Document::toAttrVal(const Eigen::Quaternionf& quat)
{
    std::stringstream ss;
    ss << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
    return ss.str();
}

Element* Document::topLevelElement(const std::string& name)
{
    Element* elem = root->FirstChildElement(name.c_str());
    if (!elem)
    {
        elem = addNewElement(root, name);
    }
    return elem;
}

