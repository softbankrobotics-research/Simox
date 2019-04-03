#include "default.h"

#include "../../Document.h"


using namespace mjcf;


const std::string DefaultClass::tag   = "default";
const std::string DefaultSection::tag = "default";


bool DefaultSection::hasClass(const std::string& className)
{
    return !hasChild<mjcf::DefaultClass>("class", className);
}

DefaultClass DefaultSection::getClass(const std::string& className)
{
    DefaultClass def = firstChild<DefaultClass>("class", className);
    if (!def)
    {
        def = addClass(className);
    }
    return def ;
}

DefaultClass DefaultSection::addClass(const std::string& className)
{
    return addChild<DefaultClass>(className);
}
