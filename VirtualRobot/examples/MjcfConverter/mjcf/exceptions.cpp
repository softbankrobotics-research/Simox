#include "exceptions.h"

#include <sstream>


using namespace VirtualRobot;


MjcfXmlLoadFileFailed::MjcfXmlLoadFileFailed(const char* errorName, const char* errorStr) : 
    MjcfConverterError(makeMsg(errorName, errorStr))
{
}

std::string MjcfXmlLoadFileFailed::makeMsg(const char* errorName, const char* errorStr)
{
    std::stringstream ss;
    ss << errorName << ": " << errorStr;
    return ss.str();
}
