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
#ifndef _VirtualRobot_TriMeshModel_h_
#define _VirtualRobot_TriMeshModel_h_

#include "../Model/Model.h"
#include "VisualizationFactory.h"
#include "TriangleFace.h"
#include "../Tools/BoundingBox.h"
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT TriMeshModel : public std::enable_shared_from_this<TriMeshModel>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TriMeshModel();

        /*struct triangle
        {
            Eigen::Vector3f vertex1;
            Eigen::Vector3f vertex2;
            Eigen::Vector3f vertex3;
        };
        TriMeshModel(std::vector <triangle>& triangles);*/

        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);
        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, 
								Eigen::Vector3f& normal,
                                Visualization::Color color1 = Visualization::Color::Gray(),
                                Visualization::Color color2 = Visualization::Color::Gray(),
                                Visualization::Color color3 = Visualization::Color::Gray());
        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, 
								Eigen::Vector4f& vertexColor1, 
								Eigen::Vector4f& vertexColor2, 
								Eigen::Vector4f& vertexColor3);
        static Eigen::Vector3f CreateNormal(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);

        void addFace(const TriangleFace& face);
        int addVertex(const Eigen::Vector3f& vertex);
        int addNormal(const Eigen::Vector3f& normal);
        int addColor(const Visualization::Color& color);
        int addColor(const Eigen::Vector4f& color);
        int addMaterial(const Visualization::PhongMaterial& material);
        void addFace(unsigned int id0, unsigned int id1, unsigned int id2);

        void clear();
        void flipVertexOrientations();
        /**
         * @brief Merges vertices that are close together (mergeThreshold).
         * Usually, vertices that are close together should be one vertex. Otherwise the mesh
         * could consist of many individual triangles.
         * All vertex ids stored in faces are updated. This function is quite efficient due to a kd-tree and an inverted face-vertex mapping.
         * @param mergeThreshold If squared Euclidan distance of two points is belong this threshold, two vertices are merged.
         * @param removeVertices If set, the vertex vextor is chekced for unused vertices. May result in  a reassembled vertex vector.
         */
        void mergeVertices(float mergeThreshold = 0.0001, bool removeVertices = true);

        /**
         * @brief fatten or shrink this trimesh. Done by moving a vertex along a normal calculated from the normals
         * of all the faces the vertex is used in.
         * @param offset All vertexes are moved about this offset in mm.
         */
        void fattenShrink(float offset);

        /*!
         * \brief removeUnusedVertices Checks if vertices are used by faces. May rearrange vertices vector!
         * @return Number of removed vertices
         */
        size_t removeUnusedVertices();

        // Overwrite all colors
        void setColor(Visualization::Color color);

        void print();
        void printNormals();
        void printVertices();
        void printFaces();
        Eigen::Vector3f getCOM();
        bool getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize);
        bool checkFacesHaveSameEdge(const TriangleFace& face1, const TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const;
        unsigned int checkAndCorrectNormals(bool inverted);

        VisualizationPtr getVisualization(bool showNormals = false, bool showLines = true);

        virtual void scale(Eigen::Vector3f& scaleFactor);
        TriMeshModelPtr clone() const;
        TriMeshModelPtr clone(Eigen::Vector3f& scaleFactor) const;

        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector3f> vertices;
        std::vector<Visualization::Color> colors;
        std::vector<TriangleFace> faces;
        std::vector<Visualization::PhongMaterial> materials;
        BoundingBox boundingBox;        
    };
} // namespace VirtualRobot

#endif /* _VirtualRobot_TriMeshModel_h_ */
