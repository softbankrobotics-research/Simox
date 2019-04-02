#pragma once

#include "Attribute.h"

namespace mjcf
{

struct VisualSection : public Element<VisualSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(VisualSection)
};

}
