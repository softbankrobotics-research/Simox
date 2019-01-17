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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"
#include "../Visualization/VisualizationFactory.h"
#include "../MathTools.h"
#include "../BoundingBox.h"
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT TriMeshModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TriMeshModel();

        struct triangle
        {
            triangle() = default;
            triangle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3) :
                vertex1(v1), vertex2(v2), vertex3(v3){}
            Eigen::Vector3f vertex1;
            Eigen::Vector3f vertex2;
            Eigen::Vector3f vertex3;
        };
        TriMeshModel(std::vector <triangle>& triangles);


        void addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3);
        void addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3, Eigen::Vector3f normal,
                                 const VisualizationFactory::Color &color1 = VisualizationFactory::Color::Gray(),
                                 const VisualizationFactory::Color &color2 = VisualizationFactory::Color::Gray(),
                                 const VisualizationFactory::Color &color3 = VisualizationFactory::Color::Gray());
        void addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3, const Eigen::Vector4f& vertexColor1, const Eigen::Vector4f& vertexColor2, const Eigen::Vector4f& vertexColor3);
        void addMesh(const TriMeshModel& mesh);
        static Eigen::Vector3f CreateNormal(const Eigen::Vector3f &vertex1, const Eigen::Vector3f &vertex2, const Eigen::Vector3f &vertex3);
        void addFace(const MathTools::TriangleFace& face);
        int addVertex(const Eigen::Vector3f& vertex);
        unsigned int addNormal(const Eigen::Vector3f& normal);
        unsigned int addColor(const VisualizationFactory::Color& color);
        unsigned int addColor(const Eigen::Vector4f& color);
        unsigned int addMaterial(const VisualizationFactory::PhongMaterial& material);
        void addFace(unsigned int id0, unsigned int id1, unsigned int id2);
        void clear();
        /**
         * @brief Checks all faces for existence of normals. Creates the normals in case they are missing.
         * @return Number of created normals
         */
        unsigned int addMissingNormals();

        /**
         * @brief  Checks all faces for existence of colors.
         * First, mergeVertices() is called. Then, if no color is found,
         * a face with the same vertex is searched and that color is used (if available).
         * Otherwise sets color to the given color.
         * @param color Default Color
         * @return Number of created colors entries
         */
        unsigned int addMissingColors(const VisualizationFactory::Color &color = VisualizationFactory::Color::Gray());

        /**
         * @brief Smoothes the normal surface by calculating the average of all face-normals of one vertex.
         * Calls first mergeVertices().
         */
        void smoothNormalSurface();
        void flipVertexOrientations();
        /**
         * @brief Merges vertices that are close together (mergeThreshold).
         * Usually, vertices that are close together should be one vertex. Otherwise the mesh
         * could consist of many individual triangles.
         * All vertex ids stored in faces are updated. This function is quite efficient due to a kd-tree and an inverted face-vertex mapping.
         * @param mergeThreshold If squared Euclidan distance of two points is belong this threshold, two vertices are merged.
         * @param removeVertices If set, the vertex vextor is chekced for unused vertices. May result in a reassembled vertex vector.
         */
        void mergeVertices(float mergeThreshold = 0.0001f, bool removeVertices = true);

        /**
         * @brief fatten or shrink this trimesh. Done by moving a vertex along a normal calculated from the normals
         * of all the faces the vertex is used in.
         * @param offset All vertexes are moved about this offset in mm.
         * @param updateNormals If true, all normals will be updated with the average normal of all normals beloning to one vertex (from multiple faces)
         */
        void fattenShrink(float offset, bool updateNormals = false);

        /*!
         * \brief removeUnusedVertices Checks if vertices are used by faces. May rearrange vertices vector!
         * @return Number of removed vertices
         */
        size_t removeUnusedVertices();

        // Overwrite all colors
        void setColor(VisualizationFactory::Color color);

        void print();
        void printNormals();
        void printVertices();
        void printFaces();
        Eigen::Vector3f getCOM();
        bool getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize);
        bool checkFacesHaveSameEdge(const MathTools::TriangleFace& face1, const MathTools::TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const;
        unsigned int checkAndCorrectNormals(bool inverted);

        virtual void scale(const Eigen::Vector3f& scaleFactor);
        TriMeshModelPtr clone() const;
        TriMeshModelPtr clone(const Eigen::Vector3f& scaleFactor) const;

        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector3f> vertices;
        std::vector<VisualizationFactory::Color> colors;
        std::vector<MathTools::TriangleFace> faces;
        std::vector<VisualizationFactory::PhongMaterial> materials;
        BoundingBox boundingBox;        
    };
} // namespace VirtualRobot

