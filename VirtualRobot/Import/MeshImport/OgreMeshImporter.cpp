#include "OgreMeshImporter.h"

//#include <tinyxml.h>
#include <boost/filesystem.hpp>
#include "../../XML/rapidxml.hpp"
#include "../../XML/BaseIO.h"

// Mostly copied from Gazeob/RViz

namespace VirtualRobot
{

Ogre::Entity *OgreMeshImporter::load(const std::string &filename, const std::string &name, Ogre::SceneManager *sceneManager)
{
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(filename, aiProcess_SortByPType | aiProcess_GenNormals | aiProcess_Triangulate | aiProcess_GenUVCoords | aiProcess_FlipUVs);
    if(!scene)
    {
        std::cout << "Invalid scene" << std::endl;
        return NULL;
    }

    if(!scene->HasMeshes())
    {
        std::cout << "Scene does not contain any meshes" << std::endl;
        return NULL;
    }

    std::vector<Ogre::MaterialPtr> materials;
    loadMaterials(filename, name, scene, materials);

    Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    Ogre::AxisAlignedBox aabb(Ogre::AxisAlignedBox::EXTENT_NULL);
    float radius = 0.0f;
    float scale = getMeshUnitRescale(name);
    buildMesh(scene, scene->mRootNode, mesh, aabb, radius, scale, materials);

    std::cout << mesh->getBounds().getMaximum() << std::endl;
    std::cout << mesh->getBounds().getMinimum() << std::endl;

    mesh->_setBounds(aabb);
    mesh->_setBoundingSphereRadius(radius);
    mesh->buildEdgeList();

    mesh->load();

    size_t vertex_count;
    Ogre::Vector3* vertices;
    size_t index_count;
    unsigned long* indices;
    getMeshInformation(mesh.get(), vertex_count, vertices, index_count, indices);

    std::cout << "Mesh imported: " << vertex_count << " vertices, " << index_count << " indices" << std::endl;

    /*for(int i = 0; i < vertex_count; i++)
    {
        std::cout << i << ": <" << vertices[i].x << ", " << vertices[i].y << ", " << vertices[i].z << ">" << std::endl;
    }*/

    return sceneManager->createEntity(mesh);
}

void OgreMeshImporter::loadMaterials(const std::string &filename, const std::string &name, const aiScene *scene, std::vector<Ogre::MaterialPtr> &material_table_out)
{
    for (uint32_t i = 0; i < scene->mNumMaterials; i++)
    {
        std::stringstream ss;
        ss << name << "Material" << i;
        Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, true);
        material_table_out.push_back(mat);

        Ogre::Technique* tech = mat->getTechnique(0);
        Ogre::Pass* pass = tech->getPass(0);

        aiMaterial *amat = scene->mMaterials[i];

        Ogre::ColourValue diffuse(1.0, 1.0, 1.0, 1.0);
        Ogre::ColourValue specular(1.0, 1.0, 1.0, 1.0);
        Ogre::ColourValue ambient(0, 0, 0, 1.0);

        for (uint32_t j=0; j < amat->mNumProperties; j++)
        {
            aiMaterialProperty *prop = amat->mProperties[j];
            std::string propKey = prop->mKey.data;

            if (propKey == "$tex.file")
            {
                aiString texName;
                aiTextureMapping mapping;
                uint32_t uvIndex;
                amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

                // Assume textures are in paths relative to the mesh
                std::string texture_path = boost::filesystem::path(filename).parent_path().string() + "/" + texName.data;
                loadTexture(texture_path);
                Ogre::TextureUnitState* tu = pass->createTextureUnitState();
                tu->setTextureName(texture_path);
            }
            else if (propKey == "$clr.diffuse")
            {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
                diffuse = Ogre::ColourValue(clr.r, clr.g, clr.b);
            }
            else if (propKey == "$clr.ambient")
            {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);
                ambient = Ogre::ColourValue(clr.r, clr.g, clr.b);
            }
            else if (propKey == "$clr.specular")
            {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
                specular = Ogre::ColourValue(clr.r, clr.g, clr.b);
            }
            else if (propKey == "$clr.emissive")
            {
                aiColor3D clr;
                amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
                mat->setSelfIllumination(clr.r, clr.g, clr.b);
            }
            else if (propKey == "$clr.opacity")
            {
                float o;
                amat->Get(AI_MATKEY_OPACITY, o);
                diffuse.a = o;
            }
            else if (propKey == "$mat.shininess")
            {
                float s;
                amat->Get(AI_MATKEY_SHININESS, s);
                mat->setShininess(s);
            }
            else if (propKey == "$mat.shadingm")
            {
                int model;
                amat->Get(AI_MATKEY_SHADING_MODEL, model);
                switch(model)
                {
                    case aiShadingMode_Flat:
                        mat->setShadingMode(Ogre::SO_FLAT);
                        break;
                    case aiShadingMode_Phong:
                        mat->setShadingMode(Ogre::SO_PHONG);
                        break;
                    case aiShadingMode_Gouraud:
                    default:
                        mat->setShadingMode(Ogre::SO_GOURAUD);
                        break;
                }
            }
        }

        int mode = aiBlendMode_Default;
        amat->Get(AI_MATKEY_BLEND_FUNC, mode);
        switch(mode)
        {
            case aiBlendMode_Additive:
                mat->setSceneBlending(Ogre::SBT_ADD);
                break;
            case aiBlendMode_Default:
            default:
            {
                if (diffuse.a < 0.99)
                {
                    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
                }
                else
                {
                    pass->setSceneBlending(Ogre::SBT_REPLACE);
                }
            }
            break;
        }

    mat->setAmbient(ambient * 0.5);
    mat->setDiffuse(diffuse);
    specular.a = diffuse.a;
    mat->setSpecular(specular);
    }
}

void OgreMeshImporter::loadTexture(const std::string& resource_path)
{
    if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path))
    {
        std::ifstream file(resource_path);
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        if (content.size() != 0)
        {
            Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(content.data(), content.size()));
            Ogre::Image image;
            std::string extension = boost::filesystem::extension(boost::filesystem::path(resource_path));

            if (extension[0] == '.')
            {
                extension = extension.substr(1, extension.size() - 1);
            }

            try
            {
                image.load(stream, extension);
                Ogre::TextureManager::getSingleton().loadImage(resource_path, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
            }
            catch (Ogre::Exception& e)
            {
                std::cout << "Could not load texture [%s]: %s" << resource_path.c_str() << e.what() << std::endl;
            }
        }
    }
}

void OgreMeshImporter::buildMesh(const aiScene *scene, const aiNode *node, const Ogre::MeshPtr &mesh, Ogre::AxisAlignedBox &aabb, float &radius, const float scale, std::vector<Ogre::MaterialPtr> &material_table)
{
    if(!node)
    {
        return;
    }

    aiMatrix4x4 transform = node->mTransformation;
    aiNode *pnode = node->mParent;
    while(pnode)
    {
        // Don't convert to y-up orientation, which is what the root node in
        // Assimp does
        if(pnode->mParent != NULL)
        {
            transform = pnode->mTransformation * transform;
        }
        pnode = pnode->mParent;
    }

    aiMatrix3x3 rotation(transform);
    aiMatrix3x3 inverse_transpose_rotation(rotation);
    inverse_transpose_rotation.Inverse();
    inverse_transpose_rotation.Transpose();

    for(uint32_t i = 0; i < node->mNumMeshes; i++)
    {
        aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

        Ogre::SubMesh* submesh = mesh->createSubMesh();
        submesh->useSharedVertices = false;
        submesh->vertexData = new Ogre::VertexData();
        Ogre::VertexData* vertex_data = submesh->vertexData;
        Ogre::VertexDeclaration* vertex_decl = vertex_data->vertexDeclaration;

        size_t offset = 0;

        // positions
        vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
        offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

        // normals
        if (input_mesh->HasNormals())
        {
            vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
            offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
        }

        // texture coordinates (only support 1 for now)
        if (input_mesh->HasTextureCoords(0))
        {
            vertex_decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
            offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
        }

        // todo vertex colors

        // allocate the vertex buffer
        vertex_data->vertexCount = input_mesh->mNumVertices;
        Ogre::HardwareVertexBufferSharedPtr vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(0),
                                                                              vertex_data->vertexCount,
                                                                              Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                                                                              false);

        vertex_data->vertexBufferBinding->setBinding(0, vbuf);
        float* vertices = static_cast<float*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

        // Add the vertices
        for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
        {
            aiVector3D p = input_mesh->mVertices[j];
            p *= transform;
            p *= scale;
            *vertices++ = p.x;
            *vertices++ = p.y;
            *vertices++ = p.z;

            Ogre::Vector3 v(p.x, p.y, p.z);
            aabb.merge(v);
            float dist = v.length();
            if (dist > radius)
            {
                radius = dist;
            }

            if (input_mesh->HasNormals())
            {
                aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
                n.Normalize();
                *vertices++ = n.x;
                *vertices++ = n.y;
                *vertices++ = n.z;
            }

            if (input_mesh->HasTextureCoords(0))
            {
                *vertices++ = input_mesh->mTextureCoords[0][j].x;
                *vertices++ = input_mesh->mTextureCoords[0][j].y;
            }
        }

        // calculate index count
        submesh->indexData->indexCount = 0;
        for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
        {
            aiFace& face = input_mesh->mFaces[j];
            submesh->indexData->indexCount += face.mNumIndices;
        }

        // If we have less than 65536 (2^16) vertices, we can use a 16-bit index buffer.
        if( vertex_data->vertexCount < (1<<16) )
        {
            // allocate index buffer
            submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
                Ogre::HardwareIndexBuffer::IT_16BIT,
                submesh->indexData->indexCount,
                Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                false);

            Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
            uint16_t* indices = static_cast<uint16_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

            // add the indices
            for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
            {
                aiFace& face = input_mesh->mFaces[j];
                for (uint32_t k = 0; k < face.mNumIndices; ++k)
                {
                    *indices++ = face.mIndices[k];
                }
            }

            ibuf->unlock();
        }
        else
        {
            // Else we have more than 65536 (2^16) vertices, so we must
            // use a 32-bit index buffer (or subdivide the mesh, which
            // I'm too impatient to do right now)

            // allocate index buffer
            submesh->indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
                Ogre::HardwareIndexBuffer::IT_32BIT,
                submesh->indexData->indexCount,
                Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
                false);

            Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
            uint32_t* indices = static_cast<uint32_t*>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

            // add the indices
            for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
            {
                aiFace& face = input_mesh->mFaces[j];
                for (uint32_t k = 0; k < face.mNumIndices; ++k)
                {
                    *indices++ = face.mIndices[k];
                }
            }

            ibuf->unlock();
        }
        vbuf->unlock();

        submesh->setMaterialName(material_table[input_mesh->mMaterialIndex]->getName());
    }

    for (uint32_t i=0; i < node->mNumChildren; ++i)
    {
        buildMesh(scene, node->mChildren[i], mesh, aabb, radius, scale, material_table);
    }
}

float OgreMeshImporter::getMeshUnitRescale(const std::string& resource_path)
{
    float unit_scale(1.0);

    std::ifstream t(resource_path);
    std::string xmlString((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    // copy string content to char array
    char* y = new char[xmlString.size() + 1];
    strncpy(y, xmlString.c_str(), xmlString.size() + 1);

    try {
        rapidxml::xml_document<char> doc;    // character type defaults to char
        doc.parse<0>(y);                     // 0 means default parse flags
        rapidxml::xml_node<char>* colladaXml = doc.first_node("COLLADA", 0, false);
        if(colladaXml)
        {
            rapidxml::xml_node<>* assetXml = colladaXml->first_node("asset", 0, false);
            if(assetXml)
            {
                rapidxml::xml_node<>* unitXml = assetXml->first_node("unit", 0, false);
                if (unitXml)
                {
                    rapidxml::xml_attribute<>* attr = unitXml->first_attribute("meter", 0, false);
                    if (attr)
                    {
                        try
                        {
                            unit_scale = BaseIO::convertToFloat(attr->value());
                        } catch (...)
                        {
                            VR_WARNING << "Failed to convert unit element meter attribute to determine scaling. unit element: " << unitXml->name() << std::endl;
                        }
                    }
                }
            }
        }
    } catch(...)
    {
        VR_WARNING << "Failed to parse " << resource_path << " to get scaling information" << endl;
    }
    return unit_scale;
}

void OgreMeshImporter::getMeshInformation(const Ogre::Mesh* const mesh, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count, unsigned long* &indices,
                        const Ogre::Vector3 &position, const Ogre::Quaternion &orient, const Ogre::Vector3 &scale)
{
    bool added_shared = false;
    size_t current_offset = 0;
    size_t shared_offset = 0;
    size_t next_offset = 0;
    size_t index_offset = 0;

    vertex_count = index_count = 0;

    // Calculate how many vertices and indices we're going to need
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }
        // Add the indices
        index_count += submesh->indexData->indexCount;
    }

    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned long[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }

            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }

        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        if ( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
                                          static_cast<unsigned long>(offset);
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
}

}
