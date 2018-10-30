/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @author     Nikolaus Vahrenkamp
* @author     Adrian Knobloch
* @copyright  2010 Manfred Kroehnert
*/


#include "Visualization.h"
#include "VisualizationSet.h"
#include "SelectionGroup.h"
#include "TriMeshModel.h"

#include <fstream>

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "CoinVisualization/CoinVisualizationFactory.h"

    using GlobalFactory = VirtualRobot::CoinVisualizationFactory;
#elif Simox_USE_QT3D_VISUALIZATION
    #include "Qt3DVisualization/Qt3DVisualizationFactory.h"

    using GlobalFactory = VirtualRobot::Qt3DVisualizationFactory;
#else
    using GlobalFactory = VirtualRobot::VisualizationFactory;
#endif

namespace VirtualRobot
{
    VisualizationFactoryPtr VisualizationFactory::getInstance()
    {
        static VisualizationFactoryPtr instance(new GlobalFactory);
        return instance;
    }

    void VisualizationFactory::init(int &, char *[], const std::string &)
    {
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox) const
    {
        std::vector<VisualizationPtr> visus;

        Eigen::Matrix4f currentTransform = Eigen::Matrix4f::Identity();

        for (std::vector<Primitive::PrimitivePtr>::const_iterator it = primitives.begin(); it != primitives.end(); it++)
        {
            Primitive::PrimitivePtr p = *it;
            currentTransform *= p->transform;
            auto visu = p->getVisualization();
            visu->setGlobalPose(currentTransform);
            visus.push_back(visu);
        }

        VisualizationSetPtr ret = createVisualisationSet(visus);

        if (boundingBox)
        {
            auto bBox = ret->getBoundingBox();
            auto bBoxVisu = bBox.getVisualization(false);
            return createVisualisationSet({ret, bBoxVisu});
        }
        else
        {
            return ret;
        }
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromFile(const std::string &filename, bool boundingBox) const
    {
        if (filename.empty() || !boost::filesystem::exists(filename))
        {
            return VisualizationPtr();
        }
        std::ifstream stream(filename);
        auto visu = createVisualizationFromFile(stream, boundingBox);
        stream.close();
        return visu;
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromFile(const std::ifstream &, bool) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationSetPtr VisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        VisualizationSetPtr v(new DummyVisualizationSet(visualizations));
        v->init();
        return v;
    }

    SelectionGroupPtr VisualizationFactory::createSelectionGroup() const
    {
        return SelectionGroupPtr(new SelectionGroup);
    }

    VisualizationPtr VisualizationFactory::createBox(float, float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Vector3f &, const Eigen::Vector3f &, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width) const
    {
        Eigen::Vector3f fromVec = from.block<3, 1>(0, 3);
        Eigen::Vector3f toVec = to.block<3, 1>(0, 3);
        return createLine(fromVec, toVec, width);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f>& from, const std::vector<Eigen::Vector3f>& to, float width) const
    {
        VR_ASSERT(from.size() == to.size());
        std::vector<VisualizationPtr> visus;
        for (size_t i = 0; i<from.size(); ++i)
        {
            visus.push_back(createLine(from[i], to[i], width));
        }
        return createVisualisationSet(visus);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f> &points, float width) const
    {
        std::vector<Eigen::Vector3f> from;
        std::vector<Eigen::Vector3f> to;
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            from.push_back(points[i]);
            to.push_back(points[i + 1]);
        }
        return createLineSet(from, to, width);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width) const
    {
        VR_ASSERT(from.size() == to.size());
        std::vector<Eigen::Vector3f> fromVec;
        std::vector<Eigen::Vector3f> toVec;
        for (size_t i = 0; i < from.size(); ++i)
        {
            fromVec.push_back(from[i].block<3, 1>(0, 3));
            toVec.push_back(to[i].block<3, 1>(0, 3));
        }
        return createLineSet(fromVec, toVec, width);
    }

    VisualizationPtr VisualizationFactory::createSphere(float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const
    {
        std::vector<Eigen::Vector3f> points;

        circleCompletion = std::min<float>(1.0f, circleCompletion);
        circleCompletion = std::max<float>(-1.0f, circleCompletion);

        for (size_t i = 0; i < numberOfCircleParts; ++i)
        {
            float angle0 = static_cast<float>(i)/static_cast<float>(numberOfCircleParts) * 2 * M_PI * circleCompletion;
            float x0 = radius * cos(angle0);
            float y0 = radius * sin(angle0);
            points.push_back(Eigen::Vector3f(x0, y0, 0));
        }

        std::vector<Eigen::Vector3f> toPoints(points);
        if(circleCompletion == 1.0f || circleCompletion == -1.0f)
        {
            toPoints.push_back(toPoints.front());
        }
        else
        {
            points.erase(points.end());
        }
        toPoints.erase(toPoints.begin());


        return createLineSet(points, toPoints, width);
    }

    VisualizationPtr VisualizationFactory::createTorus(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        VR_ASSERT_MESSAGE(rings >= 4, "You need to pass in atleast 4 rings for a torus");
        VR_ASSERT_MESSAGE(sides >= 4, "You need to pass in atleast 4 sides for a torus");

        if(fabs(completion) <= 0.01)
        {
            return createVisualization();
        }
        TriMeshModelPtr triMesh = TriMeshModelPtr(new TriMeshModel());

        triMesh->addColor(VirtualRobot::Visualization::Color::Gray());

        int numVerticesPerRow = sides + 1;
        int numVerticesPerColumn = rings + 1;

        float theta = 0.0f;
        float phi = 0.0f;
        float verticalAngularStride = (float)(M_PI * 2.0f) / (float)rings;
        float horizontalAngularStride = ((float)M_PI * 2.0f) / (float)sides;

        for (int verticalIt = 0; verticalIt < numVerticesPerColumn; verticalIt++)
        {
            theta = verticalAngularStride * verticalIt * completion;

            for (int horizontalIt = 0; horizontalIt < numVerticesPerRow; horizontalIt++)
            {
                phi = horizontalAngularStride * horizontalIt;

                // position
                float x = (float)cos(theta) * (radius + tubeRadius * (float)cos(phi));
                float y = (float)sin(theta) * (radius + tubeRadius * (float)cos(phi));
                float z = tubeRadius * (float)sin(phi);
                triMesh->addVertex(Eigen::Vector3f(x, y, z));
            }
        }


        for (int verticalIt = 0; verticalIt < rings; verticalIt++)
        {
            for (int horizontalIt = 0; horizontalIt < sides; horizontalIt++)
            {
                short lt = (short)(horizontalIt + verticalIt * (numVerticesPerRow));
                short rt = (short)((horizontalIt + 1) + verticalIt * (numVerticesPerRow));

                short lb = (short)(horizontalIt + (verticalIt + 1) * (numVerticesPerRow));
                short rb = (short)((horizontalIt + 1) + (verticalIt + 1) * (numVerticesPerRow));
                TriangleFace face;
                face.normal = -completion*TriMeshModel::CreateNormal(triMesh->vertices[lt],
                                                                     triMesh->vertices[rt],
                                                                     triMesh->vertices[lb]);
                face.id1 = lt;
                face.idColor1 = 0;

                face.id2 = rt;

                face.idColor2 = 0;

                face.id3 = lb;

                face.idColor3 = 0;

                triMesh->addFace(face);


                TriangleFace face2;
                face2.normal = -completion*TriMeshModel::CreateNormal(triMesh->vertices[rt],
                                                                      triMesh->vertices[rb],
                                                                      triMesh->vertices[lb]);
                face2.id1 = rt;
                face2.idColor1 = 0;

                face2.id2 = rb;

                face2.idColor2 = 0;

                face2.id3 = lb;

                face2.idColor3 = 0;

                triMesh->addFace(face2);
            }
        }

        return triMesh->getVisualization(false, false);
    }

    VisualizationPtr VisualizationFactory::createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        completion = std::min<float>(1.0f, completion);
        completion = std::max<float>(-1.0f, completion);
        int sign = completion >= 0 ? 1 : -1;
        float torusCompletion = completion - 1.0f  /rings * sign;
        if(torusCompletion * sign < 0)
            torusCompletion = 0;
        auto torus = createTorus(radius, tubeRadius, torusCompletion);

        // create Arrow
        float angle0 = (float)(rings - 2) / rings * 2 * M_PI * completion;
        float x0 = radius * cos(angle0);
        float y0 = radius * sin(angle0);
        float angle1 = (float)(rings - 1) / rings * 2 * M_PI * completion;


        auto arrow = createArrow(Eigen::Vector3f::UnitY() * sign, 0, tubeRadius);
        Eigen::Matrix4f gp = MathTools::axisangle2eigen4f(Eigen::Vector3f(0, 0, 1), angle1);
        gp(0, 3) = x0;
        gp(1, 3) = y0;
        gp(2, 3) = 0.0f;
        arrow->setGlobalPose(gp);

        return createVisualisationSet({torus, arrow});
    }

    VisualizationPtr VisualizationFactory::createCylinder(float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createCoordSystem(std::string *text, float axisLength, float axisSize, float blockLength, bool arrow) const
    {
        std::vector<VisualizationPtr> visus;

        if (arrow)
        {
            visus.push_back(createArrow(Eigen::Vector3f::UnitX(), axisLength, axisSize));
            Eigen::Matrix4f xPos = Eigen::Matrix4f::Identity();
            xPos(0,3) = axisLength/2 + axisSize/2.f;
            visus.back()->setGlobalPose(xPos);

            visus.push_back(createArrow(Eigen::Vector3f::UnitY(), axisLength, axisSize));
            Eigen::Matrix4f yPos = Eigen::Matrix4f::Identity();
            yPos(1,3) = axisLength/2 + axisSize/2.f;
            visus.back()->setGlobalPose(yPos);

            visus.push_back(createArrow(Eigen::Vector3f::UnitZ(), axisLength, axisSize));
            Eigen::Matrix4f zPos = Eigen::Matrix4f::Identity();
            zPos(2,3) = axisLength/2 + axisSize/2.f;
            visus.back()->setGlobalPose(zPos);
        }
        else
        {
            {
                visus.push_back(createBox(axisLength, axisSize, axisSize));
                Eigen::Matrix4f xPos = Eigen::Matrix4f::Identity();
                xPos(0,3) = axisLength/2 + axisSize/2.f;
                visus.back()->setGlobalPose(xPos);

                visus.push_back(createBox(axisSize, axisLength, axisSize));
                Eigen::Matrix4f yPos = Eigen::Matrix4f::Identity();
                yPos(1,3) = axisLength/2 + axisSize/2.f;
                visus.back()->setGlobalPose(yPos);

                visus.push_back(createBox(axisSize, axisSize, axisLength));
                Eigen::Matrix4f zPos = Eigen::Matrix4f::Identity();
                zPos(2,3) = axisLength/2 + axisSize/2.f;
                visus.back()->setGlobalPose(zPos);
            }

            if (blockLength > 0.f && blockLength < axisLength)
            {
                float blockSize = axisSize + 0.5f;
                float blockWidth = 0.1f;

                if (axisSize > 10.0f)
                {
                    blockSize += axisSize / 10.0f;
                    blockWidth += axisSize / 10.0f;
                }

                unsigned int nrOfBlocks = static_cast<unsigned int>(axisLength/blockLength);
                if (axisLength - nrOfBlocks*blockLength < 0.0001 && nrOfBlocks > 0)
                {
                    nrOfBlocks--;
                }
                for (unsigned int i=1; i<=nrOfBlocks; ++i)
                {
                    visus.push_back(createBox(blockWidth, blockSize, blockSize));
                    Eigen::Matrix4f xPos = Eigen::Matrix4f::Identity();
                    xPos(0,3) = static_cast<float>(i)*blockLength + axisSize/2.f;
                    visus.back()->setGlobalPose(xPos);

                    visus.push_back(createBox(blockSize, blockWidth, blockSize));
                    Eigen::Matrix4f yPos = Eigen::Matrix4f::Identity();
                    yPos(1,3) = static_cast<float>(i)*blockLength + axisSize/2.f;
                    visus.back()->setGlobalPose(yPos);

                    visus.push_back(createBox(blockSize, blockSize, blockWidth));
                    Eigen::Matrix4f zPos = Eigen::Matrix4f::Identity();
                    zPos(2,3) = static_cast<float>(i)*blockLength + axisSize/2.f;
                    visus.back()->setGlobalPose(zPos);
                }
            }
        }
        visus[0]->setColor(Visualization::Color::Red());
        visus[1]->setColor(Visualization::Color::Green());
        visus[2]->setColor(Visualization::Color::Blue());

        if (text)
        {
            visus.push_back(createText(*text, false, 2.f, 2.f, 0.f));
        }

        auto set = createVisualisationSet(visus);
        set->setSelectionGroup(createSelectionGroup());
        return set;
    }

    VisualizationPtr VisualizationFactory::createPoint(float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationSetPtr VisualizationFactory::createPointCloud(const std::vector<Eigen::Matrix4f> &points, float radius) const
    {
        std::vector<VisualizationPtr> visus;
        for (auto& p : points)
        {
            visus.push_back(createPoint(radius));
            visus.back()->setGlobalPose(p);
        }
        return createVisualisationSet(visus);
    }

    VisualizationSetPtr VisualizationFactory::createPointCloud(const std::vector<Eigen::Vector3f> &points, float radius) const
    {
        std::vector<VisualizationPtr> visus;
        for (auto& p : points)
        {
            visus.push_back(createPoint(radius));
            Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
            gp.block<3, 1>(0, 3) = p;
            visus.back()->setGlobalPose(gp);
        }
        return createVisualisationSet(visus);
    }

    VisualizationPtr VisualizationFactory::createTriMeshModel(const TriMeshModelPtr&) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createPolygon(const std::vector<Eigen::Vector3f> &) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createPlane(const Eigen::Vector3f &, const Eigen::Vector3f &, float, const std::string &) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createArrow(const Eigen::Vector3f &, float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createText(const std::string &, bool, float, float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createCone(float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createEllipse(float, float, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &, float, float, bool) const
    {
        //TODO implement using primitives
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createConvexHull2DVisualization(const MathTools::ConvexHull2DPtr &hull, const MathTools::Plane &p, const Eigen::Vector3f &offset) const
    {
        std::vector<Eigen::Vector3f> cvHull3d;

        for (size_t u = 0; u < hull->vertices.size(); u++)
        {
            Eigen::Vector3f pt3d = MathTools::planePoint3D(hull->vertices[u], p);
            pt3d += offset;
            cvHull3d.push_back(pt3d);
        }

        return VisualizationFactory::getInstance()->createPolygon(cvHull3d);
    }

    VisualizationPtr VisualizationFactory::createVisualization() const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    void VisualizationFactory::cleanup()
    {
        ;
    }

    std::string VisualizationFactory::getVisualizationType() const
    {
        return "dummy";
    }

} // namespace VirtualRobot
