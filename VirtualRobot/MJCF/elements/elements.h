#pragma once

#include "actuator.h"
#include "asset.h"
#include "body.h"
#include "compiler.h"
#include "contact.h"
#include "default.h"
#include "equality.h"
#include "keyframe.h"
#include "option.h"
#include "sensor.h"
#include "size.h"
#include "statistic.h"
#include "tendon.h"
#include "visual.h"


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


#undef mjcf_AttributeFullNoDef
#undef mjcf_AttributeFullDef

#undef mjcf_AttributeReq
#undef mjcf_AttributeOpt
#undef mjcf_AttributeDef

#undef mjcf_AttributeKwReq
#undef mjcf_AttributeKwOpt
#undef mjcf_AttributeKwDef

#undef mjcf_BoolAttributeReq
#undef mjcf_BoolAttributeOpt
#undef mjcf_BoolAttributeDef
    
#undef mjcf_IntAttributeReq
#undef mjcf_IntAttributeOpt
#undef mjcf_IntAttributeDef
    
#undef mjcf_FloatAttributeReq
#undef mjcf_FloatAttributeOpt
#undef mjcf_FloatAttributeDef

#undef mjcf_StringAttributeReq
#undef mjcf_StringAttributeOpt
#undef mjcf_StringAttributeDef
#undef mjcf_StringAttributeDefEmpty

#undef mjcf_Vector2fAttributeDef
#undef mjcf_Vector3fAttributeReq
#undef mjcf_Vector3fAttributeOpt
#undef mjcf_Vector3fAttributeDef
    
#undef mjcf_NameAttribute
#undef mjcf_ClassAttribute
#undef mjcf_GroupAttribute

#undef mjcf_RgbAttributeDef
#undef mjcf_RgbaAttributeDef
    
#undef mjcf_PosAttribute
#undef mjcf_QuatAttribute
#undef mjcf_PoseAttributes
