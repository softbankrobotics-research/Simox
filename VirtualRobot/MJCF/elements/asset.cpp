#include "asset.h"

#include "../Document.h"


using namespace mjcf;


const std::string Texture::tag      = "texture";
const std::string Material::tag     = "material";
const std::string Mesh::tag         = "mesh";
const std::string AssetSection::tag = "asset";


Texture AssetSection::addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2)
{
    Texture texSkybox = addChild<Texture>();

    texSkybox.type = "skybox";
    texSkybox.builtin = "gradient";
    texSkybox.width = 128;
    texSkybox.height = 128;
    texSkybox.rgb1 = rgb1;
    texSkybox.rgb2 = rgb2;

    return texSkybox;
}

Texture AssetSection::addTextureFile(const std::string& name, const std::string& file, 
                                     bool vflip, bool hflip)
{
    Texture texture = addChild<Texture>();
    texture.name = name;
    texture.file = file;
    texture.type = "2d";
    texture.vflip = vflip;
    texture.hflip = hflip;
    return texture;
}

Mesh AssetSection::addMesh(const std::string& name, const std::string& file)
{
    Mesh mesh = addChild<Mesh>();
    mesh.name = name;
    mesh.file = file;
    return mesh;
}

Material AssetSection::addMaterial(const std::string& name, const std::string& textureName)
{
    Material material = addChild<Material>();
    material.name = name;
    material.texture = textureName;
    return material;
}
