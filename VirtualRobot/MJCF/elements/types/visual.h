#pragma once

#include "../core/Attribute.h"

namespace mjcf
{

struct VisualSection : public Element<VisualSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(VisualSection)
};

}
