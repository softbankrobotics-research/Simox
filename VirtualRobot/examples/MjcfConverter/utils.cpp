#include "utils.h"

#include <algorithm>


namespace VirtualRobot
{

void assertElementIs(mjcf::Element* elem, const char* tag)
{
    assert(std::strcmp(elem->Value(), tag) == 0);
}

void assertElementIs(mjcf::Element* elem, const std::string& tag)
{
    assert(std::strcmp(elem->Value(), tag.c_str()) == 0);
}

void assertElementIsBody(mjcf::Element* elem)
{
    assertElementIs(elem, "body");
}


bool hasElement(mjcf::Element* elem, const std::string& elemName)
{
    return elem->FirstChildElement(elemName.c_str()) != nullptr;
}

bool hasMass(mjcf::Element* body)
{
    assertElementIsBody(body);
    return hasElement(body, "geom") || hasElement(body, "inertial");
}

Eigen::Vector3f strToVec(const char* string)
{
    Eigen::Vector3f v;
    sscanf(string, "%f %f %f", &v(0), &v(1), &v(2));
    return v;
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


}
