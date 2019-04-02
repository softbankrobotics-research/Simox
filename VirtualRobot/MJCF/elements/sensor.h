#pragma once

#include "Attribute.h"

namespace mjcf
{

struct SensorSection : public Element<SensorSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(SensorSection)
};

}
