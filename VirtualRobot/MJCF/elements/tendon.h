#pragma once

#include "core/Attribute.h"

namespace mjcf
{


struct TendonSection : public Element<TendonSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(TendonSection)
};


}
