#pragma once

#include <stdexcept>

namespace mjcf
{

    class MjcfError : public std::runtime_error
    {
    public:
        MjcfError(const std::string& message);
    };


    /// Indicates an error in load/saveFile.
    class MjcfIOError : public MjcfError
    {
    public:
        MjcfIOError(const std::string& message);
    };

    
    /// Indicates that an attribute without default was not set when it was 
    /// attempted to read.
    class AttribNotSetError : public MjcfError
    {
    public:
        AttribNotSetError(const std::string& name);
    };
    
    /// Indicates that an attribute without default was not set when it was 
    /// attempted to read.
    class ParseAttributeError : public MjcfError
    {
    public:
        ParseAttributeError(const std::string& source, const std::type_info& targetType,
                            const std::string& reason);
    };
    


}
