#pragma once

#include "core/Attribute.h"

namespace mjcf
{

struct SensorSection : public Element<SensorSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(SensorSection)
};

}
