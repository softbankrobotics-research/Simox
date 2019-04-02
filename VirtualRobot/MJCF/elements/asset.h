#pragma once

#include "core/Attribute.h"


namespace mjcf
{

struct Texture : public Element<Texture>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Texture)
    
    mjcf_NameAttribute(Texture);
    mjcf_StringAttributeDef(Texture, type, "cube");
    mjcf_StringAttributeOpt(Texture, file);
    
    mjcf_AttributeDef(Texture, Eigen::Vector2i, gridsize, Eigen::Vector2i::Ones());
    mjcf_StringAttributeDef(Texture, gridlayout, "............");
  
    mjcf_StringAttributeDef(Texture, builtin, "none");
    mjcf_StringAttributeDef(Texture, mark, "none");
    mjcf_RgbAttributeDef(Texture, markrgb, Eigen::Vector3f::Zero());
    mjcf_FloatAttributeDef(Texture, random, 0.01f);
    
    mjcf_IntAttributeDef(Texture, width, 0);
    mjcf_IntAttributeDef(Texture, height, 0);
  
    mjcf_RgbAttributeDef(Texture, rgb1, 0.8f * Eigen::Vector3f::Ones());
    mjcf_RgbAttributeDef(Texture, rgb2, 0.5f * Eigen::Vector3f::Ones());
    
    mjcf_BoolAttributeDef(Texture, hflip, false);
    mjcf_BoolAttributeDef(Texture, vflip, true);
};


struct Mesh : public Element<Mesh>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Mesh)
    
    mjcf_NameAttribute(Mesh);
    mjcf_ClassAttribute(Mesh);
    
    mjcf_StringAttributeOpt(Mesh, file);
    
    mjcf_Vector3fAttributeDef(Mesh, scale,  Eigen::Vector3f::Ones());
    mjcf_BoolAttributeDef(Mesh, smoothnormal, false);
    
    mjcf_Vector3fAttributeDef(Mesh, refpos,  Eigen::Vector3f::Ones());
    mjcf_AttributeDef(Mesh, Eigen::Quaternionf, refquat, Eigen::Quaternionf::Identity());
};


struct Material : public Element<Material>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(Material)
    
    mjcf_NameAttribute(Material);
    mjcf_ClassAttribute(Material);
    
    mjcf_StringAttributeOpt(Material, texture);
    
    mjcf_AttributeDef(Material, Eigen::Vector2f, texrepeat, Eigen::Vector2f::Ones());
    mjcf_BoolAttributeDef(Material, texuniform, false);
    
    mjcf_FloatAttributeDef(Material, emission, 0);
    mjcf_FloatAttributeDef(Material, specular, 0);
    mjcf_FloatAttributeDef(Material, shininess, 0);
    mjcf_FloatAttributeDef(Material, reflectance, 0);
    mjcf_RgbaAttributeDef(Material, rgba, Eigen::Vector4f::Ones());
};



struct AssetSection : public Element<AssetSection>
{
    static const std::string tag;
    mjcf_ElementDerivedConstructors(AssetSection)
    
    Texture addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2);    
    
    Texture addTextureFile(const std::string& name, const std::string& file,
                           bool vflip = false, bool hflip = false);
    
    /// Add a mesh asset with a name and a file.
    Mesh addMesh(const std::string& name, const std::string& file);
    
    Material addMaterial(const std::string& name, const std::string& textureName);
};

}
