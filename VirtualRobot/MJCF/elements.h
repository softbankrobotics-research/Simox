#pragma once

#include "elements/core/AnyElement.h"
#include "elements/types/actuator.h"
#include "elements/types/asset.h"
#include "elements/types/body.h"
#include "elements/types/compiler.h"
#include "elements/types/contact.h"
#include "elements/types/default.h"
#include "elements/types/equality.h"
#include "elements/types/keyframe.h"
#include "elements/types/option.h"
#include "elements/types/sensor.h"
#include "elements/types/size.h"
#include "elements/types/statistic.h"
#include "elements/types/tendon.h"
#include "elements/types/visual.h"


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

#undef mjcf_SolimpAttribute
#undef mjcf_SolrefAttribute
