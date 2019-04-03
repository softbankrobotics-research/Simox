#pragma once

#include "core/Attribute.h"

namespace mjcf
{

struct CompilerSection : public Element<CompilerSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(CompilerSection)
    
    mjcf_FloatAttributeDef(CompilerSection, boundmass, 0);
    mjcf_FloatAttributeDef(CompilerSection, boundinertia, 0);
    mjcf_FloatAttributeDef(CompilerSection, settotalmass, -1);
    
    mjcf_BoolAttributeDef(CompilerSection, balanceinertia, false);
    mjcf_BoolAttributeDef(CompilerSection, strippath, false);
    
    // local, global
    mjcf_StringAttributeDef(CompilerSection, coordinate, "local");
    // degree, radian (default "degree" for MJCF, always "radian" for URDF)
    mjcf_StringAttributeDef(CompilerSection, angle, "degree");
    mjcf_BoolAttributeDef(CompilerSection, fitaabb, false);
    mjcf_StringAttributeDef(CompilerSection, eulerseq, "xyz");

    mjcf_StringAttributeOpt(CompilerSection, meshdir);
    mjcf_StringAttributeOpt(CompilerSection, texturedir);
    // Default: false for MJCF, true for URDF
    mjcf_BoolAttributeDef(CompilerSection, discardvisual, false);

    mjcf_BoolAttributeDef(CompilerSection, convexhull, true);
    mjcf_BoolAttributeDef(CompilerSection, userthread, true);
    
    // Default: false for MJCF, true for URDF
    mjcf_BoolAttributeDef(CompilerSection, fusestatic, false);

    // false, true, auto
    mjcf_StringAttributeDef(CompilerSection, inertiafromgeom, "auto");
    mjcf_AttributeDef(CompilerSection, Eigen::Vector2i, inertiagrouprange, Eigen::Vector2i(0, 5));
};

}
