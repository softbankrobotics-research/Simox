#pragma once

#include "core/Attribute.h"

namespace mjcf
{

struct CompilerSection : public Element<CompilerSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(CompilerSection)
};

}
