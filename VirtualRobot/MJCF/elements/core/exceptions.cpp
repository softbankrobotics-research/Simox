#include "exceptions.h"

#include <typeinfo>


namespace mjcf
{

MjcfError::MjcfError(const std::string& message) : std::runtime_error(message)
{}

MjcfIOError::MjcfIOError(const std::string& message) : MjcfError (message)
{}

AttribNotSetError::AttribNotSetError(const std::string& name) : 
    MjcfError("Attrib '" + name + "' (without default) not set.")
{}

ParseAttributeError::ParseAttributeError(
        const std::string& source, 
        const std::type_info& targetType, 
        const std::string& reason) :
    MjcfError("Could not parse attribute string '" + source + " to " + targetType.name() + ".\n"
              "Reason: " + reason)
{}


}
