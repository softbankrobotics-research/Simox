#pragma once

#include "../core/Attribute.h"


namespace mjcf
{

/// @see http://www.mujoco.org/book/XMLreference.html#mujoco
struct MujocoRoot : public Element<MujocoRoot>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(MujocoRoot)
    
    mjcf_StringAttributeDef(MujocoRoot, model, "MuJoCo Model");
};


/// @see http://www.mujoco.org/book/XMLreference.html#include
struct Include : public Element<Include>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Include)
    
    mjcf_StringAttributeReq(Include, file);
};

}
