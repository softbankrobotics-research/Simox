#pragma once

#include "../core/Attribute.h"
#include "body.h"


namespace mjcf
{


struct ContactPair : public Element<ContactPair>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ContactPair)
    
    mjcf_NameAttribute(ContactPair);
    mjcf_ClassAttribute(ContactPair);
    
    mjcf_StringAttributeReq(ContactPair, geom1);
    mjcf_StringAttributeReq(ContactPair, geom2);
    
    mjcf_IntAttributeDef(ContactPair, condim, 3);
    
    mjcf_AttributeDef(ContactPair, Eigen::Vector5f, friction, condimDefault());
    
    mjcf_FloatAttributeDef(ContactPair, margin, 0);
    mjcf_FloatAttributeDef(ContactPair, gap, 0);
    
private:
    static Eigen::Vector5f condimDefault();
};


struct ContactExclude : public Element<ContactExclude>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ContactExclude)
    
    mjcf_NameAttribute(ContactExclude);
    mjcf_StringAttributeReq(ContactExclude, body1);
    mjcf_StringAttributeReq(ContactExclude, body2);
};


struct ContactSection : public Element<ContactSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ContactSection)
    
    /// Add a conact/pair element between the given geoms.
    ContactPair addPair(const Geom& geom1, const Geom& geom2);
    /// Add a conact/pair element between the given geoms.
    ContactPair addPair(const std::string& geom1Name, const std::string& geom2Name);
    
    /// Add a conact/exclude element between the given bodies.
    ContactExclude addExclude(const Body& body1, const Body& body2);
    /// Add a conact/exclude element between the given bodies.
    ContactExclude addExclude(const std::string& body1Name, const std::string& body2Name);
};


}
