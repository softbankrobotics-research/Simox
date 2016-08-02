/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OgreMeshImporter_h_
#define _VirtualRobot_OgreMeshImporter_h_

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/postprocess.h>

#include <OGRE/Ogre.h>
#include "../../VirtualRobot.h"

namespace VirtualRobot
{

class VIRTUAL_ROBOT_IMPORT_EXPORT OgreMeshImporter
{
    public:
        static Ogre::Entity *load(const std::string &filename, const std::string &name, Ogre::SceneManager *sceneManager);

    protected:
        static void loadMaterials(const std::string &filename, const std::string &name, const aiScene *scene, std::vector<Ogre::MaterialPtr> &material_table_out);
        static void buildMesh(const aiScene* scene, const aiNode* node, const Ogre::MeshPtr& mesh, Ogre::AxisAlignedBox& aabb, float& radius, const float scale, std::vector<Ogre::MaterialPtr>& material_table);
        static float getMeshUnitRescale(const std::string& resource_path);
        static void loadTexture(const std::string& resource_path);

        static void getMeshInformation(const Ogre::Mesh* const mesh, size_t &vertex_count, Ogre::Vector3* &vertices, size_t &index_count, unsigned long* &indices,
                                const Ogre::Vector3 &position = Ogre::Vector3::ZERO, const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY, const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);
};

}

#endif
