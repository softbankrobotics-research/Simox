#pragma once

#include "Attribute.h"

namespace mjcf
{

struct StatisticSection : public Element<StatisticSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(StatisticSection)
};


}
