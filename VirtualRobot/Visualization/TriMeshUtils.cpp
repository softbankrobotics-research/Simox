/*
 * This file is part of ArmarX.
 *
 * Copyright (C) 2011-2017, High Performance Humanoid Technologies (H2T), Karlsruhe Institute of Technology (KIT), all rights reserved.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    ArmarX
 * @author     Mirko Waechter( mirko.waechter at kit dot edu)
 * @date       2018
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */
#include "TriMeshUtils.h"

namespace VirtualRobot {

void TriMeshUtils::CreateBoxTriangles(std::vector<TriMeshModel::triangle> &triangles, const Eigen::Matrix4f &globalPose, float width, float height, float depth)
{
    auto w = width/2;
    auto h = height/2;
    auto d = depth/2;

    auto transf = [&](Eigen::Vector3f& pos)
    {
        pos = globalPose.block<3,3>(0,0) * pos + globalPose.block<3,1>(0,3);
    };

    static const float rawVertices[] = {
        -1.0f,-1.0f,-1.0f, // triangle 1 : begin
        -1.0f,-1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f, // triangle 1 : end
        1.0f, 1.0f,-1.0f, // triangle 2 : begin
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f, // triangle 2 : end
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f
    };
    triangles.resize(12*3);
    std::array<Eigen::Vector3f, 3> vertices;
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 3; ++j) {
            int offset = i*9+j*3;
            auto& v = vertices.at(j);
            v.x() = rawVertices[offset+0]*w;
            v.y() = rawVertices[offset+1]*h;
            v.z() = rawVertices[offset+2]*d;
            transf(v);
        }
        triangles.at(i) = {vertices.at(0), vertices.at(1), vertices.at(2)};
    }
}

std::vector<TriMeshModel::triangle> TriMeshUtils::CreateBoxTriangles(const Eigen::Matrix4f &globalPose, float width, float height, float depth)
{
    std::vector<TriMeshModel::triangle> triangles;
    CreateBoxTriangles(triangles, globalPose, width, height, depth);
    return triangles;
}

TriMeshModelPtr TriMeshUtils::CreateBox(const Eigen::Matrix4f &globalPose, float width, float height, float depth, const VisualizationFactory::Color &color,
                                        const std::vector<VisualizationFactory::Color>& colors)
{
    TriMeshModelPtr mesh(new TriMeshModel());
    auto triangles = CreateBoxTriangles(globalPose, width, height, depth);
    size_t i = 0;
    for(auto& t : triangles)
    {
        decltype (color) selectedColor = i < colors.size() ? colors.at(i) : color;
        mesh->addTriangleWithFace(t.vertex1, t.vertex2, t.vertex3,
                                  TriMeshModel::CreateNormal(t.vertex1, t.vertex2, t.vertex3),
                                  selectedColor, selectedColor, selectedColor);
        i++;
    }
    return mesh;
}

TriMeshModelPtr TriMeshUtils::CreateSparseBoxGrid(const Eigen::Matrix4f &globalPose, const std::vector<Eigen::Vector3f> &positions,
                                                  float width, float height, float depth,
                                                  const VisualizationFactory::Color &color, const std::vector<VisualizationFactory::Color>& colors)
{

    TriMeshModelPtr mesh(new TriMeshModel());
    mesh->vertices.reserve(positions.size()*12*3);
    mesh->colors.reserve(positions.size()*12*3);
    mesh->normals.reserve(positions.size()*12);
    mesh->faces.reserve(positions.size()*12);
    size_t i = 0;
    std::vector<TriMeshModel::triangle> triangles;
    for(auto& pos : positions)
    {
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3,1>(0,3) = pos;
        pose = globalPose * pose;
        CreateBoxTriangles(triangles, pose, width, height, depth);
        decltype (color)& selectedColor = i < colors.size() ? colors.at(i) : color;
        for(auto& t : triangles)
        {
            mesh->addTriangleWithFace(t.vertex1, t.vertex2, t.vertex3,
                                      TriMeshModel::CreateNormal(t.vertex1, t.vertex2, t.vertex3),
                                      selectedColor, selectedColor, selectedColor);
        }
        i++;
    }
    return mesh;
}

} // namespace VirtualRobot
