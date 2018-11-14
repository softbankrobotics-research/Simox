#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot 
{
namespace mjcf
{

bool isElement(const XMLElement* elem, const char* tag);
bool isElement(const XMLElement& elem, const char* tag);
bool isElement(const XMLElement* elem, const std::string& tag);
bool isElement(const XMLElement& elem, const std::string& tag);

bool hasElementChild(const XMLElement* elem, const std::string& elemName);

bool hasMass(const XMLElement* body);


// Convert to MJCF XML attribute format.
std::string toAttr(bool b);
template <int dim>
std::string toAttr(const Eigen::Matrix<float, dim, 1>& v);
std::string toAttr(const Eigen::Quaternionf& v);

// Convert from MJCF XML attribute.
Eigen::Vector2f strToVec2(const char* string);
Eigen::Vector3f strToVec(const char* string);
Eigen::Quaternionf strToQuat(const char* string);

// Get lenght of common prefix of two strings (was used for mergin body names).
std::size_t commonPrefixLength(const std::string& a, const std::string& b);


// Assert that isElement(elem, tag).
template <class StringT>
void assertElementIs(const XMLElement* elem, const StringT tag);

// Assert that isElement(elem, "body").
void assertElementIsBody(const XMLElement* elem);




// DEFINITIONS of templated methods


template <int dim>
std::string toAttr(const Eigen::Matrix<float, dim, 1>& v)
{
    static const Eigen::IOFormat iofVector {7, 0, "", " ", "", "", "", ""};
    
    std::stringstream ss;
    ss << v.format(iofVector);
    return ss.str();
}

template <class StringT>
void assertElementIs(const XMLElement* elem, const StringT tag)
{
    assert(isElement(elem, tag));
}

}
}

