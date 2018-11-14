#include "utils.h"

#include <algorithm>
#include <map>


namespace VirtualRobot
{
namespace mjcf
{

ActuatorType toActuatorType(std::string str)
{
    static const std::map<std::string, ActuatorType> map {
        {"motor",    ActuatorType::MOTOR},
        {"position", ActuatorType::POSITION},
        {"velocity", ActuatorType::VELOCITY}
    };
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    return map.at(str);
}



bool isElement(const XMLElement* elem, const char* tag)
{
    return isElement(*elem, tag);
}

bool isElement(const XMLElement* elem, const std::string& tag)
{
    return isElement(*elem, tag);
}

bool isElement(const XMLElement& elem, const char* tag)
{
    return std::strcmp(elem.Value(), tag) == 0;
}

bool isElement(const XMLElement& elem, const std::string& tag)
{
    return std::strcmp(elem.Value(), tag.c_str()) == 0;
}


bool hasElementChild(const XMLElement* elem, const std::string& elemName)
{
    return elem->FirstChildElement(elemName.c_str()) != nullptr;
}

bool hasMass(const XMLElement* body)
{
    assertElementIsBody(body);
    return hasElementChild(body, "geom") || hasElementChild(body, "inertial");
}



std::string toAttr(bool b)
{
    static const std::string strings[] = { "false", "true" };
    return strings[int(b)];
}

std::string toAttr(const Eigen::Quaternionf& quat)
{
    std::stringstream ss;
    ss << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
    return ss.str();
}


Eigen::Vector2f strToVec2(const char* string)
{
    Eigen::Vector2f v;
    sscanf(string, "%f %f", &v(0), &v(1));
    return v;
}

Eigen::Vector3f strToVec(const char* string)
{
    Eigen::Vector3f v;
    sscanf(string, "%f %f %f", &v(0), &v(1), &v(2));
    return v;
}

Eigen::Quaternionf strToQuat(const char* string)
{
    Eigen::Quaternionf q;
    sscanf(string, "%f %f %f %f", &q.w(), &q.x(), &q.y(), &q.z());
    return q;
}


std::size_t commonPrefixLength(const std::string& a, const std::string& b)
{
    const std::string* smaller = &a;
    const std::string* bigger = &b;
    if (b.size() < a.size())
    {
        std::swap(smaller, bigger);
    }
    
    auto mismatch = std::mismatch(smaller->begin(), smaller->end(), 
                                  bigger->begin()).first;
    return std::size_t(std::distance(smaller->begin(), mismatch));
}


void assertElementIsBody(const XMLElement* elem)
{
    assertElementIs(elem, "body");
}

bool hasAttr(const XMLElement* elem, const std::string& attrName)
{
    return hasAttr(*elem, attrName);
}

bool hasAttr(const XMLElement& elem, const std::string& attrName)
{
    return elem.Attribute(attrName.c_str());
}

bool isAttr(const XMLElement* elem, const std::string& attrName, const std::string& attrValue)
{
    return isAttr(*elem, attrName, attrValue);
}

bool isAttr(const XMLElement& elem, const std::string& attrName, const std::string& attrValue)
{
    return hasAttr(elem, attrName) && elem.Attribute(attrName.c_str()) == attrValue;
}




}
}
