#pragma once

#include <stdexcept>


namespace VirtualRobot
{

class MjcfConverterError : public std::runtime_error
{
public:
    // inherit constructor
    using std::runtime_error::runtime_error;
};


/**
 * @brief Indicates that tinyxml2::XMLDocument::LoadFile() failed.
 */
class MjcfXmlLoadFileFailed : public MjcfConverterError
{
public:
    MjcfXmlLoadFileFailed(const char* errorName, const char* errorStr);
    
private:
    static std::string makeMsg(const char* errorName, const char* errorStr);
    
};









}
