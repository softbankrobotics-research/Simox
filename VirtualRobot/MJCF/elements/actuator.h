#pragma once

#include "core/Attribute.h"


namespace mjcf
{

static Eigen::Vector6f gearDefault()
{
    Eigen::Vector6f v;
    v << 1, 0, 0, 0, 0, 0;
    return v;
}


#define mjcf_ActuatorAttributes(Derived)    \
    mjcf_NameAttribute(Derived);    \
    mjcf_ClassAttribute(Derived);   \
    \
    mjcf_GroupAttribute(Derived);   \
    \
    mjcf_BoolAttributeDef(Derived, ctrllimited, false);     \
    mjcf_BoolAttributeDef(Derived, forcelimited, false);    \
    \
    mjcf_Vector2fAttributeDef(Derived, ctrlrange, Eigen::Vector2f::Zero());     \
    mjcf_Vector2fAttributeDef(Derived, forcerange, Eigen::Vector2f::Zero());    \
    mjcf_Vector2fAttributeDef(Derived, lengthrange, Eigen::Vector2f::Zero());   \
    \
    mjcf_AttributeDef(Derived, Eigen::Vector6f, gear, gearDefault());   \
    mjcf_FloatAttributeDef(Derived, cranklength, 0);    \
    \
    mjcf_StringAttributeOpt(Derived, joint);            \
    mjcf_StringAttributeOpt(Derived, jointinparent);    \
    mjcf_StringAttributeOpt(Derived, site);             \
    mjcf_StringAttributeOpt(Derived, tendon);           \
    mjcf_StringAttributeOpt(Derived, cranksite);        \
    mjcf_StringAttributeOpt(Derived, slidersite)


struct ActuatorGeneral : public Element<ActuatorGeneral>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorGeneral)
    
    mjcf_ActuatorAttributes(ActuatorGeneral);
    
    // dyntype : [none, integrator, filter, muscle, user], 
    mjcf_StringAttributeDef(ActuatorGeneral, dyntype, "none");
    // gaintype : [fixed, muscle, user], "fixed"
    mjcf_StringAttributeDef(ActuatorGeneral, gaintype, "fixed");
    // biastype : [none, affine, muscle, user], "none"
    mjcf_StringAttributeDef(ActuatorGeneral, biastype, "none");
    
    // dynprm : real(10), "1 0 ... 0"
    mjcf_AttributeDef(ActuatorGeneral, Eigen::VectorXf, dynprm, dynprmDefault());
    // gainprm : real(10), "1 0 ... 0"
    mjcf_AttributeDef(ActuatorGeneral, Eigen::VectorXf, gainprm, gainprmDefault());
    // biasprm : real(10), "1 0 ... 0"
    mjcf_AttributeDef(ActuatorGeneral, Eigen::VectorXf, biasprm, biasprmDefault());
    
    
private:
    static Eigen::VectorXf dynprmDefault();
    static Eigen::VectorXf gainprmDefault();
    static Eigen::VectorXf biasprmDefault();
    
};

struct ActuatorMotor : public Element<ActuatorMotor>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorMotor)
    
    mjcf_ActuatorAttributes(ActuatorMotor);
};

struct ActuatorPosition : public Element<ActuatorPosition>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorPosition)
    
    mjcf_ActuatorAttributes(ActuatorPosition);
    mjcf_FloatAttributeDef(ActuatorPosition, kp, 1);
};

struct ActuatorVelocity : public Element<ActuatorVelocity>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorVelocity)
    
    mjcf_ActuatorAttributes(ActuatorVelocity);
    mjcf_FloatAttributeDef(ActuatorVelocity, kv, 1);
};

struct ActuatorCylinder : public Element<ActuatorCylinder>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorCylinder)
    
    mjcf_ActuatorAttributes(ActuatorCylinder);
    mjcf_FloatAttributeDef(ActuatorCylinder, timeconst, 1);
    mjcf_FloatAttributeDef(ActuatorCylinder, area, 1);
    mjcf_FloatAttributeOpt(ActuatorCylinder, diameter);
    mjcf_Vector3fAttributeDef(ActuatorCylinder, bias, Eigen::Vector3f::Zero());
};

struct ActuatorMuscle : public Element<ActuatorMuscle>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorMuscle)
    
    mjcf_ActuatorAttributes(ActuatorMuscle);
    
    mjcf_Vector2fAttributeDef(ActuatorMuscle, timeconst, timeconstDefault());
    mjcf_Vector2fAttributeDef(ActuatorMuscle, range, rangeDefault());
    
    mjcf_FloatAttributeDef(ActuatorMuscle, force, -1);
    mjcf_FloatAttributeDef(ActuatorMuscle, scale, 200);
    mjcf_FloatAttributeDef(ActuatorMuscle, lmin, 0.5);
    mjcf_FloatAttributeDef(ActuatorMuscle, lmax, 1.6f);
    mjcf_FloatAttributeDef(ActuatorMuscle, vmax, 1.5);
    mjcf_FloatAttributeDef(ActuatorMuscle, fpmax, 1.3f);
    mjcf_FloatAttributeDef(ActuatorMuscle, vvmax, 1.2f);
    
private:
    static Eigen::Vector2f timeconstDefault();
    static Eigen::Vector2f rangeDefault();
};


struct ActuatorSection : public Element<ActuatorSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(ActuatorSection)
    
    ActuatorMotor addMotor(const std::string& jointName);
    ActuatorPosition addPosition(const std::string& joint, float kp = -1);
    ActuatorVelocity addVelocity(const std::string& joint, float kv = -1);
    
};

}
