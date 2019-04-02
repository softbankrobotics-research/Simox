#pragma once

#include "Attribute.h"
#include "mjcf_utils.h"

namespace mjcf
{


struct OptionFlag : public Element<OptionFlag>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(OptionFlag)
};

struct OptionSection : public Element<OptionSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(OptionSection)
    
    
    /// Get (or create) the option/flag child.
    OptionFlag flag();
    
    
    mjcf_FloatAttributeDef(OptionSection, timestep, 0.002f);
    mjcf_FloatAttributeDef(OptionSection, impratio, 1);
    
    mjcf_Vector3fAttributeDef(OptionSection, gravity, Eigen::Vector3f(0, 0, -9.81f));
    
    mjcf_Vector3fAttributeDef(OptionSection, wind, Eigen::Vector3f::Zero());
    mjcf_Vector3fAttributeDef(OptionSection, magnetic, Eigen::Vector3f(0, -0.5, 0));
    
    mjcf_FloatAttributeDef(OptionSection, density, 0);
    mjcf_FloatAttributeDef(OptionSection, viscosity, 0);
    
    mjcf_FloatAttributeDef(OptionSection,             o_margin, 0);
    mjcf_Vector2fAttributeDef(OptionSection,          o_solref, Eigen::Vector2f(0.02, 1));
    mjcf_AttributeDef(OptionSection, Eigen::Vector5f, o_solimp, o_solimp_default());
    
    // [Euler, RK4]
    mjcf_StringAttributeDef(OptionSection, integrator, "Euler");
    // [all, predefined, dynamic]
    mjcf_StringAttributeDef(OptionSection, collision, "all");
    // [pyramidal, elliptic]
    mjcf_StringAttributeDef(OptionSection, cone, "pyramidal");
    // [dense, sparse, auto]
    mjcf_StringAttributeDef(OptionSection, jacobian, "auto");
    // [PGS, CG, Newton]
    mjcf_StringAttributeDef(OptionSection, solver, "Newton");
    
    mjcf_IntAttributeDef(OptionSection,   iterations, 100);
    mjcf_FloatAttributeDef(OptionSection, tolerance,  1e-8f);
    mjcf_IntAttributeDef(OptionSection,   noslip_iterations, 0);
    mjcf_FloatAttributeDef(OptionSection, noslip_tolerance,  1e-6f);
    mjcf_IntAttributeDef(OptionSection,   mpr_iterations, 50);
    mjcf_FloatAttributeDef(OptionSection, mpr_tolerance,  1e-6f);
    
private:
    
    static Eigen::Vector5f o_solimp_default()
    {
        Eigen::Vector5f v;
        v << 0.9f, 0.95f, 0.001f, 0.5, 2;
        return v;
    }
};



}
