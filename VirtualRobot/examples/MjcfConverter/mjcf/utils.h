#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot 
{


bool isElement(const mjcf::Element* elem, const char* tag);
bool isElement(const mjcf::Element& elem, const char* tag);
bool isElement(const mjcf::Element* elem, const std::string& tag);
bool isElement(const mjcf::Element& elem, const std::string& tag);

bool hasElementChild(const mjcf::Element* elem, const std::string& elemName);

bool hasMass(const mjcf::Element* body);


// Convert to MJCF XML attribute format.
std::string toAttr(bool b);
template <int dim>
std::string toAttr(const Eigen::Matrix<float, dim, 1>& v);
std::string toAttr(const Eigen::Quaternionf& v);

Eigen::Vector3f strToVec(const char* string);
Eigen::Quaternionf strToQuat(const char* string);

std::size_t commonPrefixLength(const std::string& a, const std::string& b);


template <class StringT>
void assertElementIs(const mjcf::Element* elem, const StringT tag);

void assertElementIsBody(const mjcf::Element* elem);




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
void assertElementIs(const mjcf::Element* elem, const StringT tag)
{
    assert(isElement(elem, tag));
}

}

