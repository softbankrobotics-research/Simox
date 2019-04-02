#pragma once

#include "core/Attribute.h"


namespace mjcf
{

#define mjcf_EqualityAttribs(Derived, entityName) \
    mjcf_NameAttribute(Derived);    \
    mjcf_ClassAttribute(Derived);   \
    mjcf_BoolAttributeDef(Derived, active, true);   \
    mjcf_StringAttributeReq(Derived, entityName ## 1);    \
    mjcf_StringAttributeOpt(Derived, entityName ## 2)



struct EqualityConnect : public Element<EqualityConnect>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualityConnect)
    
    mjcf_EqualityAttribs(EqualityConnect, body);
    
    mjcf_Vector3fAttributeReq(EqualityConnect, anchor);
};


struct EqualityWeld : public Element<EqualityWeld>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualityWeld)

    mjcf_EqualityAttribs(EqualityWeld, body);
    
    mjcf_AttributeDef(EqualityWeld, Eigen::Vector7f, relpose, relposeDefault());
    
private:
    static Eigen::Vector7f relposeDefault();
    
};


struct EqualityJoint : public Element<EqualityJoint>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualityJoint)

    mjcf_EqualityAttribs(EqualityJoint, joint);
    
    mjcf_AttributeDef(EqualityJoint, Eigen::Vector5f, polycoef, polycoefDefault());
    
private:
    static Eigen::Vector5f polycoefDefault();
    
};


struct EqualityTendon : public Element<EqualityTendon>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualityTendon)

    mjcf_EqualityAttribs(EqualityTendon, tendon);
    
    mjcf_AttributeDef(EqualityTendon, Eigen::Vector5f, polycoef, polycoefDefault());
    
private:
    static Eigen::Vector5f polycoefDefault();
    
};

struct EqualityDistance : public Element<EqualityDistance>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualityDistance)

    mjcf_EqualityAttribs(EqualityDistance, geom);
    
    mjcf_FloatAttributeDef(EqualityDistance, distance, 0);
};


struct EqualitySection : public Element<EqualitySection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(EqualitySection)
    
    EqualityWeld addWeld(const std::string& name, const std::string& body1, 
                         const std::string& body2);
    
};


#undef mjcf_EqualityAttribs
}
