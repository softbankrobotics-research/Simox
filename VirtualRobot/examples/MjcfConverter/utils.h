#pragma once

#include "MjcfDocument.h"


namespace VirtualRobot 
{

void assertElementIs(mjcf::Element* elem, const char* tag);
void assertElementIs(mjcf::Element* elem, const std::string& tag);

void assertElementIsBody(mjcf::Element* elem);


bool hasElement(mjcf::Element* elem, const std::string& elemName);

bool hasMass(mjcf::Element* body);


Eigen::Vector3f strToVec(const char* string);

}

