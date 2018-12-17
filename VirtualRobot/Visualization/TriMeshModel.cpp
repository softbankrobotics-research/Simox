/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#include "TriMeshModel.h"
#include "../VirtualRobot.h"
#include "../DataStructures/nanoflann.hpp"
#include "../DataStructures/KdTreePointCloud.h"
#include<Eigen/Geometry>

#include <algorithm>
#include <fstream>
#include <iomanip>


namespace VirtualRobot
{


    TriMeshModel::TriMeshModel()
    = default;

    TriMeshModel::TriMeshModel(std::vector <triangle>& triangles)
    {
        for (auto & triangle : triangles)
        {
            addTriangleWithFace(triangle.vertex1, triangle.vertex2, triangle.vertex3);
        }
    }



    /**
     * This method adds the vertices \p vertex1,
     * \p vertex2 and \p vertex3 to TriMeshModel::vertices and creates a new
     * TriangleFace instance which is added to TriMeshModel::faces.
     *
     * \param vertex1 first vertex to use in the calculation
     * \param vertex2 second vertex to use in the calculation
     * \param vertex3 third vertex to use in the calculation
     */
    void TriMeshModel::addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3)
    {
        Eigen::Vector3f normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        addTriangleWithFace(vertex1, vertex2, vertex3, normal);
    }

    void TriMeshModel::addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3, Eigen::Vector3f normal, const VisualizationFactory::Color &color1, const VisualizationFactory::Color &color2, const VisualizationFactory::Color &color3)
    {
//        VR_INFO << vertex1 << "\n\n" << vertex2 << "\n\n" << vertex3 << "\n\n" << std::endl;
        this->addVertex(vertex1);
        this->addVertex(vertex2);
        this->addVertex(vertex3);

        this->addColor(color1);
        this->addColor(color2);
        this->addColor(color3);

        if (normal.norm() < 1e-10 || std::isnan(normal[0]) || std::isnan(normal[1]) || std::isnan(normal[2]))
        {
            normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        }
        else
        {
            normal.normalize();
        }

        // create face
        MathTools::TriangleFace face;
        face.id1 = this->vertices.size() - 3;
        face.id2 = this->vertices.size() - 2;
        face.id3 = this->vertices.size() - 1;

        face.idColor1 = this->colors.size() - 3;
        face.idColor2 = this->colors.size() - 2;
        face.idColor3 = this->colors.size() - 1;

        face.normal = normal;
        if (std::isnan(face.normal[0]) || std::isnan(face.normal[1]) || std::isnan(face.normal[2]))
        {
            VR_ERROR << "*** NANNNNNNNNNNNNNNNNNNNNN" << endl;
        }

        this->addFace(face);
    }

    void TriMeshModel::addTriangleWithFace(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3, const Eigen::Vector4f& vertexColor1, const Eigen::Vector4f& vertexColor2, const Eigen::Vector4f& vertexColor3)
    {
        Eigen::Vector3f normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        VisualizationFactory::Color color1(vertexColor1(0), vertexColor1(1), vertexColor1(2), vertexColor1(4));
        VisualizationFactory::Color color2(vertexColor2(0), vertexColor2(1), vertexColor2(2), vertexColor2(4));
        VisualizationFactory::Color color3(vertexColor3(0), vertexColor3(1), vertexColor3(2), vertexColor3(4));
        addTriangleWithFace(vertex1, vertex2, vertex3, normal, color1, color2, color3);
    }

    void TriMeshModel::addMesh(const TriMeshModel &mesh)
    {


        for(auto & face : mesh.faces)
        {
            addTriangleWithFace(mesh.vertices.at(face.id1), mesh.vertices.at(face.id2), mesh.vertices.at(face.id3),
                                face.normal, mesh.colors.at(face.idColor1), mesh.colors.at(face.idColor2), mesh.colors.at(face.idColor3));
        }
//        VR_INFO << " size after : " << vertices.size() << std::endl;

    }


    /**
     * This method creates the normal belonging to the vertices \p vertex1,
     * \p vertex2 and \p vertex3.
     *
     * \param vertex1 first vertex to use in the calculation
     * \param vertex2 second vertex to use in the calculation
     * \param vertex3 third vertex to use in the calculation
     *
     * \return normal vector
     */
    Eigen::Vector3f TriMeshModel::CreateNormal(const Eigen::Vector3f& vertex1, const Eigen::Vector3f& vertex2, const Eigen::Vector3f& vertex3)
    {
        static bool warningPrinted = false;
        // calculate normal
        Eigen::Vector3f v1v2 = vertex2 - vertex1;
        Eigen::Vector3f v1v3 = vertex3 - vertex1;
        Eigen::Vector3f normal = v1v2.cross(v1v3);

        float l = normal.norm();

        if (l < 1e-10)
        {
            if (!warningPrinted)
            {
                VR_INFO << ": Warning: tiny normal found in TriMeshModel. This error is printed only once!\n";
                warningPrinted = true;
            }
        } else
        {
            normal /= l;
        }

        return normal;
    }


    /**
     * This method adds a face to the internal data structure TriMeshModel::faces.
     */
    void TriMeshModel::addFace(const MathTools::TriangleFace& face)
    {
        faces.push_back(face);
    }

    void TriMeshModel::addFace(unsigned int id0, unsigned int id1, unsigned int id2)
    {
        MathTools::TriangleFace f;
        f.id1 = id0;
        f.id2 = id1;
        f.id3 = id2;
        addFace(f);
    }


    /**
    * This method adds a vertex to the internal data structure TriMeshModel::vertices.
    */
    int TriMeshModel::addVertex(const Eigen::Vector3f& vertex)
    {
        if (std::isnan(vertex[0]) || std::isnan(vertex[1]) || std::isnan(vertex[2]))
        {
            VR_ERROR << "NAN vertex added!!!" << endl;
            return -1;
        }
        vertices.push_back(vertex);
        boundingBox.addPoint(vertex);
        return vertices.size() - 1;
    }

    /**
    * This method adds a normal to the internal data structure TriMeshModel::normals.
    */
    unsigned int TriMeshModel::addNormal(const Eigen::Vector3f& normal)
    {
        normals.push_back(normal);
        return normals.size() - 1;

    }

    /**
     * This method adds a color to the internal data structure TriMeshModel::colors
     */
    unsigned int TriMeshModel::addColor(const VisualizationFactory::Color& color)
    {
        colors.push_back(color);
        return colors.size() - 1;

    }

    /**
     * This method converts and adds a color to the internal data structure TriMeshModel::colors
     */
    unsigned int TriMeshModel::addColor(const Eigen::Vector4f& color)
    {
        return addColor(VisualizationFactory::Color(color(0), color(1), color(2), color(3)));
    }

    /**
     * This method converts and adds a color to the internal data structure TriMeshModel::materials
     */
    unsigned int TriMeshModel::addMaterial(const VisualizationFactory::PhongMaterial& material)
    {
        materials.push_back(material);
        return materials.size() - 1;
    }


    /**
     * This method clears the internal data structures TriMeshModel::faces and
     * TriMeshModel::vertices.
     */
    void TriMeshModel::clear()
    {
        vertices.clear();
        colors.clear();
        faces.clear();
        materials.clear();
        boundingBox.clear();
    }

    unsigned int TriMeshModel::addMissingNormals()
    {
        int counter = 0;
        for(auto& face : faces)
        {
            if(face.idNormal1 != UINT_MAX && face.idNormal2 != UINT_MAX && face.idNormal3 != UINT_MAX)
            {
                continue;
            }

            if (face.normal.norm() < 1e-10f || std::isnan(face.normal[0]) || std::isnan(face.normal[1]) || std::isnan(face.normal[2]))
            {
                face.normal = CreateNormal(vertices.at(face.id1), vertices.at(face.id2), vertices.at(face.id3));
            }
            auto normalId = UINT_MAX;
            if(face.idNormal1 == UINT_MAX)
            {
                normalId = face.idNormal1 = addNormal(face.normal);
                counter++;
            }
            if(face.idNormal2 == UINT_MAX)
            {
                if(normalId == UINT_MAX)
                    face.idNormal2 = normalId;
                else
                {
                    normalId = face.idNormal2 = addNormal(face.normal);
                    counter++;
                }
            }
            if(face.idNormal3 == UINT_MAX)
            {
                if(normalId == UINT_MAX)
                    face.idNormal3 = normalId;
                else
                {
                    normalId = face.idNormal3 = addNormal(face.normal);
                    counter++;
                }
            }
        }
        return counter;
    }

    unsigned int TriMeshModel::addMissingColors(const VisualizationFactory::Color &color)
    {
        mergeVertices();
        int counter = 0;

        auto getColorId = [&](unsigned int vertexId)
        {
            for(auto& face : faces)
            {
                if(face.id1 == vertexId && face.idColor1 != UINT_MAX)
                    return face.idColor1;
                if(face.id2 == vertexId && face.idColor2 != UINT_MAX)
                    return face.idColor2;
                if(face.id3 == vertexId && face.idColor3 != UINT_MAX)
                    return face.idColor3;
            }
            return addColor(color);
        };
        for(auto& face : faces)
        {
            if(face.idColor1 != UINT_MAX && face.idColor2 != UINT_MAX && face.idColor3 != UINT_MAX)
            {
                continue;
            }

            auto colorId = UINT_MAX;
            if(face.idColor1 == UINT_MAX)
            {
                colorId = face.idColor1 = getColorId(face.id1);
                counter++;
            }
            if(face.idColor2 == UINT_MAX)
            {
                colorId = face.idColor2 = getColorId(face.id2);
                counter++;
            }
            if(face.idColor3 == UINT_MAX)
            {
                colorId = face.idColor3 = getColorId(face.id3);
                counter++;
            }
        }
        return counter;
    }

    void TriMeshModel::smoothNormalSurface()
    {
        mergeVertices();
        addMissingNormals();
        fattenShrink(0.0f, true);
    }


    /**
     * This method calls TriangleFace::flipOrientation() on each entry in
     * TriMeshModel::faces.
     */
    void TriMeshModel::flipVertexOrientations()
    {
        std::for_each(faces.begin(), faces.end(), std::mem_fun_ref(&MathTools::TriangleFace::flipOrientation));
    }




    void TriMeshModel::mergeVertices(float mergeThreshold, bool removeVertices)
    {
        int size = vertices.size();
        int faceCount = faces.size();
        std::vector<std::set<MathTools::TriangleFace*>> vertex2FaceMap(size);
        for (int j = 0; j < faceCount; ++j)
        {
            MathTools::TriangleFace& face = faces.at(j);
            vertex2FaceMap[face.id1].insert(&faces.at(j));
            vertex2FaceMap[face.id2].insert(&faces.at(j));
            vertex2FaceMap[face.id3].insert(&faces.at(j));
        }
#if 1
        PointCloud<float> cloud;
        cloud.pts.reserve(size);
        for (int i = 0; i < size; ++i)
        {
            cloud.pts.emplace_back(PointCloud<float>::Point{vertices.at(i)[0],
                                                            vertices.at(i)[1],
                                                            vertices.at(i)[2]});
        }
        using num_t = float;
        // construct a kd-tree index:
        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
            PointCloud<num_t>,
            3 /* dim */
            > my_kd_tree_t;

        my_kd_tree_t   index(3 /*dim*/, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
        index.buildIndex();



        const num_t search_radius = static_cast<num_t>(mergeThreshold);
        std::vector<std::pair<size_t,num_t> >   ret_matches;

        nanoflann::SearchParams params;
        num_t query_pt[3];
        params.sorted = false;
        for (int i = 0; i < size; ++i)
        {
            auto& p1 = vertices.at(i);
            query_pt[0] = p1[0];
            query_pt[1] = p1[1];
            query_pt[2] = p1[2];
            const size_t nMatches = index.radiusSearch(&query_pt[0],search_radius, ret_matches, params);
            for (size_t k = 0; k < nMatches; ++k)
            {
                int foundIndex = ret_matches.at(k).first;
                auto& faces = vertex2FaceMap[foundIndex];
                for(MathTools::TriangleFace* facePtr : faces)
                {
                    bool found = false;
                    if(facePtr->id1 == foundIndex)
                    {
                        facePtr->id1 = i;
                        found = true;
                    }
                    if(facePtr->id2 == foundIndex)
                    {
                        facePtr->id2 = i;
                        found = true;
                    }
                    if(facePtr->id3 == foundIndex)
                    {
                        facePtr->id3 = i;
                        found = true;
                    }
                    if(found)
                        vertex2FaceMap[i].insert(facePtr);
                }
            }
        }

#else
        std::vector<bool> deleted(size, false);
        for (int i = 0; i < size; ++i)
        {
            auto& p1 = vertices.at(i);
            for (int k = 0; k < size; ++k)
            {
                if(k == i || deleted.at(k))
                    continue;
                auto &p2 = vertices.at(k);
                if((p1-p2).norm() < mergeThreshold)
                {
//                    deleted.at(k) = true;
                    for(MathTools::TriangleFace* facePtr : vertex2FaceMap[k])
                    {
                        bool found = false;
                        if(facePtr->id1 == k)
                        {
                            facePtr->id1 = i;
                            found = true;
                        }
                        if(facePtr->id2 == k)
                        {
                            facePtr->id2 = i;
                            found = true;
                        }
                        if(facePtr->id3 == k)
                        {
                            facePtr->id3 = i;
                            found = true;
                        }
                        if(found)
                            vertex2FaceMap[i].insert(facePtr);
                    }
                }
            }
        }
#endif

        if (removeVertices)
            removeUnusedVertices();
    }

    size_t TriMeshModel::removeUnusedVertices()
    {
        auto size = vertices.size();
        auto faceCount = faces.size();
        std::vector<std::set<MathTools::TriangleFace*>> vertex2FaceMap(size);
        for (size_t j = 0; j < faceCount; ++j)
        {
            MathTools::TriangleFace& face = faces.at(j);
            vertex2FaceMap[face.id1].insert(&faces.at(j));
            vertex2FaceMap[face.id2].insert(&faces.at(j));
            vertex2FaceMap[face.id3].insert(&faces.at(j));
        }

        std::vector<Eigen::Vector3f> newVertices;
        std::map<unsigned int, unsigned int> oldNewIndexMap;

        for (size_t i=0; i<vertex2FaceMap.size(); i++ )
        {
            std::set<MathTools::TriangleFace*> &fs = vertex2FaceMap.at(i);
            if (fs.size()>0)
            {
                Eigen::Vector3f &v = vertices.at(i);
                newVertices.push_back(v);
                oldNewIndexMap[i] = (unsigned int)newVertices.size()-1;
            }
        }

        // update faceID
        for (size_t j = 0; j < faceCount; ++j)
        {
            MathTools::TriangleFace& face = faces.at(j);
            face.id1 = oldNewIndexMap[face.id1];
            face.id2 = oldNewIndexMap[face.id2];
            face.id3 = oldNewIndexMap[face.id3];
        }
        size_t removed = vertices.size() - newVertices.size();

        vertices.swap(newVertices);
        return removed;
    }

    void TriMeshModel::fattenShrink(float offset, bool updateNormals)
    {
        size_t i;
        size_t size = this->faces.size();
        std::vector<Eigen::Vector3f> averageNormals(vertices.size(), Eigen::Vector3f::Zero());
        for (i = 0; i < size; ++i)
        {
            MathTools::TriangleFace& face = faces.at(i);
            auto& p1 = vertices.at(face.id1);
            auto& p2 = vertices.at(face.id2);
            auto& p3 = vertices.at(face.id3);
            auto normal1 = face.idNormal1 < normals.size() ? normals.at(face.idNormal1) : face.normal;
            auto normal2 = face.idNormal2 < normals.size() ? normals.at(face.idNormal2) : face.normal;
            auto normal3 = face.idNormal3 < normals.size() ? normals.at(face.idNormal3) : face.normal;
            if(std::isnan(face.normal[0]) || std::isnan(face.normal[1]) || std::isnan(face.normal[2]))
                std::cout << "NAN in face " << i << " : " << face.normal << std::endl;
            if(std::isnan(normal1[0]) || std::isnan(normal1[1]) || std::isnan(normal1[2]))
                std::cout << "NAN in normal1 " << i << " : " << face.normal << std::endl;
            if(std::isnan(normal1[0]) || std::isnan(normal2[1]) || std::isnan(normal2[2]))
                std::cout << "NAN in normal2 " << i << " : " << face.normal << std::endl;
            if(std::isnan(normal3[0]) || std::isnan(normal3[1]) || std::isnan(normal3[2]))
                std::cout << "NAN in normal3 " << i << " : " << face.normal << std::endl;

            if(normal1.norm() > 0)
            {
                // weight with angle of the triangle: bigger angle -> higher area -> higher influence
                Eigen::Vector3f p1p2 = -p1 + p2;
                Eigen::Vector3f p1p3 = -p1 + p3;
                float angle = MathTools::getAngle(p1p2, p1p3);
                averageNormals.at(face.id1) += normal1.normalized() * angle;
            }
            if(normal2.norm() > 0)
            {
                Eigen::Vector3f p2p1 = -p2 + p1;
                Eigen::Vector3f p2p3 = -p2 + p3;
                float angle = MathTools::getAngle(p2p1, p2p3);

                averageNormals.at(face.id2) += normal2.normalized() * angle;
            }
            if(normal3.norm() > 0)
            {
                Eigen::Vector3f p3p2 = -p3 + p2;
                Eigen::Vector3f p3p1 = -p3 + p1;
                float angle = MathTools::getAngle(p3p2, p3p1);

                averageNormals.at(face.id3) += normal3.normalized() * angle;
            }
        }
        for(auto & n : averageNormals)
        {
            n.normalize();
        }


        for (i = 0; i < vertices.size(); ++i)
        {
            auto& p = vertices.at(i);
            auto& normal = averageNormals.at(i);
            if(averageNormals.at(i).norm() > 0)
            {
                if(std::isnan(normal[0]) || std::isnan(normal[1]) || std::isnan(normal[2]))
                    std::cout << "NAN in " << i << " : " << normal  << std::endl;
                p += normal * offset;
            }
        }

        if(updateNormals)
        {
            normals.clear();
            normals.reserve(size);
            for (i = 0; i < size; ++i)
            {
                MathTools::TriangleFace& face = faces.at(i);
                face.idNormal1 = addNormal(averageNormals.at(face.id1));
                face.idNormal2 = addNormal(averageNormals.at(face.id2));
                face.idNormal3 = addNormal(averageNormals.at(face.id3));
            }
        }


    }

    void TriMeshModel::setColor(VisualizationFactory::Color color)
    {
        colors.clear();
        for (size_t i=0; i<vertices.size(); i++)
            colors.push_back(color);
    }


    /**
     * This method calculates the center of mass by accumulating all vertices and
     * dividing the sum by the number of vertices.
     */
    Eigen::Vector3f TriMeshModel::getCOM()
    {
        Eigen::Vector3f centerOfMass = Eigen::Vector3f::Zero();

        // accumulate all vertices
        std::vector<Eigen::Vector3f>::size_type i = 0;

        for (; i < vertices.size(); i++)
        {
            centerOfMass += vertices[i];
        }

        // divide by the number of vertices
        if (!vertices.empty())
        {
            centerOfMass /= (float)vertices.size();
        }

        return centerOfMass;
    }

    bool TriMeshModel::getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize)
    {
        if (vertices.size() == 0)
        {
            return false;
        }

        storeMinSize = vertices[0];
        storeMaxSize = vertices[0];

        // go through all vertices
        std::vector<Eigen::Vector3f>::size_type i = 0;

        for (; i < vertices.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (vertices[i][j] < storeMinSize[j])
                {
                    storeMinSize[j] = vertices[i][j];
                }

                if (vertices[i][j] > storeMaxSize[j])
                {
                    storeMaxSize[j] = vertices[i][j];
                }
            }
        }

        return true;
    }

    /**
     * This method checks if the faces \p face1 and \p face2 share one common edge.
     *
     * \param face1 first TriangleFace to use in comparison
     * \param face2 second TriangleFace to use in comparison
     * \param commonVertexIds contains a list of
     *
     * \return true if both faces share the same edge and false if not
     */
    bool TriMeshModel::checkFacesHaveSameEdge(const MathTools::TriangleFace& face1, const MathTools::TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const
    {
        commonVertexIds.clear();
        Eigen::Vector2i vertexIds;
        const Eigen::Vector3f& face1Id1 = vertices[face1.id1];
        const Eigen::Vector3f& face1Id2 = vertices[face1.id2];
        const Eigen::Vector3f& face1Id3 = vertices[face1.id3];
        const Eigen::Vector3f& face2Id1 = vertices[face2.id1];
        const Eigen::Vector3f& face2Id2 = vertices[face2.id2];
        const Eigen::Vector3f& face2Id3 = vertices[face2.id3];

        // compare id1 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id1 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(1, 1));
        }

        if (face1Id1 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(1, 2));
        }

        if (face1Id1 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(1, 3));
        }

        // compare id2 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id2 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(2, 1));
        }

        if (face1Id2 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(2, 2));
        }

        if (face1Id2 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(2, 3));
        }

        // compare id3 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id3 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(3, 1));
        }

        if (face1Id3 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(3, 2));
        }

        if (face1Id3 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(3, 3));
        }

        // if both faces share 2 vertices they also share
        // one edge which goes from one vertex to the other
        return (2 == commonVertexIds.size());
    }


    /**
     * This method checks if all normals of the model point inwards or outwards and
     * flippes the faces which have a wrong orientation.
     * \param inverted inverts the check if set to true
     * \return the number of flipped faces
     */
    unsigned int TriMeshModel::checkAndCorrectNormals(bool inverted)
    {
        MathTools::TriangleFace* f1, *f2;
        int a1, a2, b1, b2;
        int flippedFacesCount = 0;

        // compare each vertex with every other vertex
        for (unsigned int i = 0; i < faces.size(); i++)
        {
            f1 = &(faces[i]);

            for (unsigned int j = 0; j < faces.size(); j++)
            {
                // don't compare each face with itself
                if (i == j)
                {
                    continue;
                }

                f2 = &(faces[j]);
                std::vector<std::pair<int, int> > commonVertexIds;

                if (checkFacesHaveSameEdge(*f1, *f2, commonVertexIds))
                {
                    a1 = commonVertexIds[0].first; // first common vertex id face1
                    a2 = commonVertexIds[1].first; // second common vertex id face1
                    b1 = commonVertexIds[0].second; // first common vertex id face
                    b2 = commonVertexIds[1].second; // second common vertex id face2
                    bool bAok = ((a1 == 1 && a2 == 2) || (a1 == 2 && a2 == 3) || (a1 == 3 && a2 == 1));
                    bool bBok = ((b1 == 1 && b2 == 3) || (b1 == 3 && b2 == 2) || (b1 == 2 && b2 == 1));

                    if (inverted)
                    {
                        bAok = !bAok;
                    }

                    // if both faces are not oriented the same flip f2
                    if (bAok && !bBok)
                    {
                        flippedFacesCount++;
                        f2->flipOrientation();
                    }
                    else if (!bAok &&  bBok)
                    {
                        flippedFacesCount++;
                        f2->flipOrientation();
                    }
                }
            }
        }

        return flippedFacesCount;
    }


    void TriMeshModel::print()
    {
        cout << "TriMeshModel\nNr of Faces:" << faces.size() << "\nNr of vertices:" << vertices.size() << endl;
        boundingBox.print();
    }


    void TriMeshModel::printNormals()
    {
        cout << "TriMeshModel Normals:" << endl;
        std::streamsize pr = cout.precision(2);
        for (auto & face : faces)
        {
            cout << "<" << face.normal(0) << "," << face.normal(1) << "," << face.normal(2) << ">,";
        }
        cout.precision(pr);
    }

    void TriMeshModel::printVertices()
    {
        cout << "TriMeshModel Vertices:" << endl;
        std::streamsize pr = cout.precision(2);
        for (size_t i = 0; i < vertices.size(); i++)
        {
            cout << vertices[i].transpose() << endl;
        }
        cout.precision(pr);
    }

    void TriMeshModel::printFaces()
    {
        cout << "TriMeshModel Faces (vertex indices):" << endl;
        std::streamsize pr = cout.precision(2);
        for (auto & face : faces)
        {
            cout << face.id1 << "," << face.id2 << "," << face.id3 << endl;
        }
        cout.precision(pr);
    }


    void TriMeshModel::scale(Eigen::Vector3f& scaleFactor)
    {
        if (scaleFactor(0) == 1.0f && scaleFactor(1) == 1.0f && scaleFactor(2) == 1.0f)
        {
            return;
        }

        for (size_t i = 0; i < vertices.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                vertices[i][j] *= scaleFactor(j);
            }
        }

        boundingBox.scale(scaleFactor);
    }

    VirtualRobot::TriMeshModelPtr TriMeshModel::clone() const
    {
        Eigen::Vector3f scaleFactor;
        scaleFactor << 1.0f, 1.0f, 1.0f;
        return clone(scaleFactor);
    }

    VirtualRobot::TriMeshModelPtr TriMeshModel::clone(Eigen::Vector3f& scaleFactor) const
    {
        TriMeshModelPtr r(new TriMeshModel());
        r->vertices = vertices;
        r->faces = faces;
        r->boundingBox = boundingBox;
        r->colors = colors;
        r->materials = materials;
        r->normals = normals;
        r->scale(scaleFactor);
        return r;
    }


} // namespace VirtualRobot
