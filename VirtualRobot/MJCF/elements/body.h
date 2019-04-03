#pragma once

#include "core/Attribute.h"


namespace mjcf
{


struct Inertial : public Element<Inertial>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Inertial)
    
    mjcf_PoseAttributes(Inertial);
    
    mjcf_AttributeReq(Inertial, float, mass);
    
    mjcf_AttributeOpt(Inertial, Eigen::Vector3f, diaginertia);
    mjcf_AttributeOpt(Inertial, Eigen::Vector6f, fullinertia);
    
    /// Sets diaginertia or fullinertia (as adequate) from the given matrix.
    void inertiaFromMatrix(const Eigen::Matrix3f& matrix);
};


struct Joint : public Element<Joint>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Joint)
    
    void setAxis(const Eigen::Vector3f& axis);
    
    mjcf_NameAttribute(Joint);
    mjcf_ClassAttribute(Joint);
    
    // free, ball, slide, hinge
    mjcf_StringAttributeDef(Joint, type, "hinge");
    mjcf_IntAttributeDef(Joint, group, 0);
    
    mjcf_PosAttribute(Joint);
    mjcf_Vector3fAttributeDef(Joint, axis, Eigen::Vector3f(0, 0, 1));
    
    mjcf_AttributeDef(Joint, Eigen::Vector2f, springdamper, Eigen::Vector2f::Zero());
    mjcf_FloatAttributeDef(Joint, stiffness, 0);
    
    mjcf_BoolAttributeDef(Joint, limited, false);
    mjcf_AttributeDef(Joint, Eigen::Vector2f, range, Eigen::Vector2f::Zero());
    
    mjcf_FloatAttributeDef(Joint, margin, 0);
    
    mjcf_FloatAttributeDef(Joint, ref, 0);
    mjcf_FloatAttributeDef(Joint, springref, 0);
    
    mjcf_FloatAttributeDef(Joint, armature, 0);
    mjcf_FloatAttributeDef(Joint, damping, 0);
    mjcf_FloatAttributeDef(Joint, frictionloss, 0);
};


struct FreeJoint : public Element<FreeJoint>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(FreeJoint)
    
    mjcf_NameAttribute(FreeJoint);
    mjcf_IntAttributeDef(FreeJoint, group, 0);
};


struct Geom : public Element<Geom>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Geom)
    
    mjcf_NameAttribute(Geom);
    mjcf_ClassAttribute(Geom);
    
    // plane, hfield, sphere, capsule, ellipsoid, cylinder, box, mesh
    mjcf_StringAttributeDef(Geom, type, "sphere");
    
    mjcf_IntAttributeDef(Geom, contype, 1);
    mjcf_IntAttributeDef(Geom, conaffinity, 1);
    mjcf_IntAttributeDef(Geom, condim, 3);
    
    mjcf_IntAttributeDef(Geom, group, 0);
    
    mjcf_Vector3fAttributeDef(Geom, size, Eigen::Vector3f::Zero());
    mjcf_StringAttributeOpt(Geom, material);
    mjcf_RgbaAttributeDef(Geom, rgba, Eigen::Vector4f(.5, .5, .5, 1));
    
    mjcf_Vector3fAttributeDef(Geom, friction, Eigen::Vector3f(1, 0.005f, 0.0001f));
    
    mjcf_FloatAttributeOpt(Geom, mass);
    mjcf_FloatAttributeDef(Geom, density, 1000);
    
    mjcf_FloatAttributeDef(Geom, solmix, 1);
    
    mjcf_FloatAttributeDef(Geom, marin, 0);
    mjcf_FloatAttributeDef(Geom, gap, 0);
    
    mjcf_AttributeOpt(Geom, Eigen::Vector6f, fromto);
    
    mjcf_PoseAttributes(Geom);
    
    mjcf_StringAttributeOpt(Geom, hfield);
    mjcf_StringAttributeOpt(Geom, mesh);
    
    mjcf_FloatAttributeDef(Geom, fitscale, 1);
};

struct Site : public Element<Site>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Site)
    
    mjcf_NameAttribute(Site);
    mjcf_ClassAttribute(Site);
    
    // sphere, capsule, ellipsoid, cylinder, box
    mjcf_StringAttributeDef(Site, type, "sphere");
    mjcf_IntAttributeDef(Site, group, 0);

    mjcf_StringAttributeOpt(Site, material);
    mjcf_RgbaAttributeDef(Site, rgba, Eigen::Vector4f(.5, .5, .5, 1));
    
    mjcf_Vector3fAttributeDef(Site, size, Eigen::Vector3f::Constant(0.005f));
    mjcf_AttributeOpt(Site, Eigen::Vector6f, fromto);
    
    mjcf_PoseAttributes(Site);
};

struct Camera : public Element<Camera>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Camera)
    
    mjcf_NameAttribute(Camera);
    mjcf_ClassAttribute(Camera);
    
    // fixed, track, trackcom, targetbody, targetbodycom
    mjcf_StringAttributeDef(Camera, mode, "fixed");
    mjcf_StringAttributeOpt(Camera, target); // required if targetbody or targetbodycom

    /// Vertical field of view of the camera, expressed in degrees regardless 
    /// of the global angle setting. 
    mjcf_FloatAttributeDef(Camera, fovy, 45);
    /// Inter-pupilary distance.
    mjcf_FloatAttributeDef(Camera, ipd, 0.068f);
    
    mjcf_PoseAttributes(Camera);
};

struct Light : public Element<Light>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Light)
    
    mjcf_NameAttribute(Light);
    mjcf_ClassAttribute(Light);
    
    // fixed, track, trackcom, targetbody, targetbodycom
    mjcf_StringAttributeDef(Light, mode, "fixed");
    mjcf_StringAttributeOpt(Light, target); // required if targetbody or targetbodycom

    mjcf_BoolAttributeDef(Light, directional, false);
    mjcf_BoolAttributeDef(Light, castshadow, false);
    mjcf_BoolAttributeDef(Light, active, false);
    
    mjcf_PosAttribute(Light);
    mjcf_Vector3fAttributeDef(Light, dir, - Eigen::Vector3f::UnitZ());
    
    mjcf_Vector3fAttributeDef(Light, attenuation, Eigen::Vector3f::UnitX());
    mjcf_FloatAttributeDef(Light, cutoff, 45);
    mjcf_FloatAttributeDef(Light, exponent, 10);
    
    mjcf_Vector3fAttributeDef(Light, ambient, Eigen::Vector3f::Zero());
    mjcf_Vector3fAttributeDef(Light, diffuse, Eigen::Vector3f::Constant(0.7f));
    mjcf_Vector3fAttributeDef(Light, specular, Eigen::Vector3f::Constant(0.3f));
};

struct Body : public Element<Body>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Body)
    
    bool hasMass() const;
    
    /// Add a child body element.
    Body addBody(const std::string& name = "");
    
    /// Add an inertial element.
    Inertial addInertial();
    /// Add an inertial element with given settings.
    Inertial addInertial(const Eigen::Vector3f& pos, float mass, const Eigen::Matrix3f& matrix);
    /// Add a dummy inertial element with small mass and identity inertia matrix.
    Inertial addDummyInertial();
    
    
    /// Add a joint element to a body.
    Joint addJoint();
    /// Add a free joint element to a body.
    FreeJoint addFreeJoint();

    /// Add a geom to a body, referencing a mesh.
    Geom addGeom(const std::string& type);
    /// Add a mesh geom with optional material (for texture).
    Geom addGeomMesh(const std::string& meshName, const std::string& materialName = "");
    
    
    mjcf_NameAttribute(Body);
    mjcf_StringAttributeOpt(Body, childclass);
    
    mjcf_BoolAttributeDef(Body, mocap, false);
    
    mjcf_PoseAttributes(Body);
};

struct Worldbody : public Element<Worldbody>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Worldbody)
    
    /// Add a body element.
    Body addBody(const std::string& name = "", const std::string& childClass = "");
    
    /// Add a mocap body with the given name to the worldbody.
    Body addMocapBody(const std::string& name, float geomSize);
    
};

}
