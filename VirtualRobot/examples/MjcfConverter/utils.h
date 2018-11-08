#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot 
{


bool isElement(const mjcf::Element* elem, const char* tag);
bool isElement(const mjcf::Element* elem, const std::string& tag);

bool hasElementChild(const mjcf::Element* elem, const std::string& elemName);

bool hasMass(const mjcf::Element* body);


Eigen::Vector3f strToVec(const char* string);

std::size_t commonPrefixLength(const std::string& a, const std::string& b);


template <class StringT>
void assertElementIs(const mjcf::Element* elem, const StringT tag)
{
    assert(isElement(elem, tag));
}

void assertElementIsBody(const mjcf::Element* elem);

}

