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
#include "ColorMap.h"

#include <fstream>

#ifdef Simox_USE_COIN_VISUALIZATION
    #include "CoinVisualization/CoinVisualizationFactory.h"

    using GlobalFactory = VirtualRobot::CoinVisualizationFactory;
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

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Matrix4f &, const Eigen::Matrix4f &, float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
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
        std::vector<VisualizationPtr> visus;
        for (size_t i = 0; i<points.size()-1; ++i)
        {
            visus.push_back(createLine(points[i], points[i+1], width));
        }
        return createVisualisationSet(visus);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width) const
    {
        VR_ASSERT(from.size() == to.size());
        std::vector<VisualizationPtr> visus;
        for (size_t i = 0; i<from.size(); ++i)
        {
            visus.push_back(createLine(from[i], to[i], width));
        }
        return createVisualisationSet(visus);
    }

    VisualizationPtr VisualizationFactory::createSphere(float) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createCircle(float, float, float, size_t) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createTorus(float, float, float, int, int) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
    }

    VisualizationPtr VisualizationFactory::createCircleArrow(float, float, float, int, int) const
    {
        VisualizationPtr v(new DummyVisualization);
        v->init();
        return v;
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

    VisualizationPtr VisualizationFactory::create2DHeightMap(const Eigen::MatrixXf &d, float extendCellX, float extendCellY, float heightZ, const ColorMap &cm, bool drawZeroCells, bool drawLines) const
    {
        auto nX = d.rows();
        auto nY = d.cols();

        VR_ASSERT(nX > 0);
        VR_ASSERT(nY > 0);

        //Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        float sizeX = extendCellX;
        float sizeY = extendCellY;

        /*      SoCube *cube = new SoCube();
                    cube->width = sizeX;
                    cube->depth = 1.0;
                    cube->height = sizeY;*/

        float maxEntry = 1.0f;

        if (d.maxCoeff() > 1.0f)
        {
            VR_ERROR << "Maximal coefficient must not be >1!" << endl;
        }

        auto res = createVisualisationSet({});

        for (int x = 1; x < nX; x++)
        {
            for (int y = 1; y < nY; y++)
            {
                float v1 = d(x - 1, y - 1);
                float v2 = d(x, y - 1);
                float v3 = d(x - 1, y);
                float v4 = d(x, y);
                float xPos1 = (float)(x - 1) * sizeX + 0.5f * sizeX; // center of voxel
                float xPos2 = (float)(x) * sizeX + 0.5f * sizeX; // center of voxel
                float yPos1 = (float)(y - 1) * sizeY + 0.5f * sizeY; // center of voxel
                float yPos2 = (float)y * sizeY + 0.5f * sizeY; // center of voxel
                float v = (v1 + v2 + v3 + v4) / 4.0f;

                float intensity1 = (float)v1 / maxEntry;
                float height1 = intensity1 * heightZ;
                float intensity2 = (float)v2 / maxEntry;
                float height2 = intensity2 * heightZ;
                float intensity3 = (float)v3 / maxEntry;
                float height3 = intensity3 * heightZ;
                float intensity4 = (float)v4 / maxEntry;
                float height4 = intensity4 * heightZ;

                Eigen::Vector3f p1(xPos1, yPos1, height1);
                Eigen::Vector3f p2(xPos2, yPos1, height2);
                Eigen::Vector3f p3(xPos1, yPos2, height3);
                Eigen::Vector3f p4(xPos2, yPos2, height4);
                std::vector<Eigen::Vector3f> pts;
                pts.push_back(p1);
                pts.push_back(p2);
                pts.push_back(p4);
                pts.push_back(p3);

                float intensity = (float)v;
                intensity /= maxEntry;

                if (intensity > 1.0f)
                {
                    intensity = 1.0f;
                }

                if (drawZeroCells || intensity > 0.)
                {
                    auto pol = createPolygon(pts);
                    pol->setColor(cm.getColor(intensity));
                    res->addVisualization(pol);
                    if (drawLines)
                    {
                        pts.push_back(pts[0]);
                        auto lines = createLineSet(pts, 4.f);
                        lines->setColor(Visualization::Color::Black());
                        res->addVisualization(lines);
                    }
                }
            }
        }

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 1>(0, 3) = Eigen::Vector3f(-extendCellX * nX / 2, -extendCellY * nY / 2, 0);
        res->setGlobalPose(pose);
        res->setGlobalPoseNoUpdate(Eigen::Matrix4f::Identity()); // move the global pose to center of the returned visualization
        return res;
    }

    VisualizationPtr VisualizationFactory::create2DMap(const Eigen::MatrixXf &d, float extendCellX, float extendCellY, const ColorMap cm, bool drawZeroCells, bool drawLines) const
    {
        auto nX = d.rows();
        auto nY = d.cols();

        VR_ASSERT(nX > 0);
        VR_ASSERT(nY > 0);

        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        float sizeX = extendCellX;
        float sizeY = extendCellY;


        float ro, gr, bl;

        float maxEntry = 1.0f;

        if (d.maxCoeff() > 1.0f)
        {
            VR_ERROR << "Maximal coefficient must not be >1!" << endl;
        }

        auto res = createVisualisationSet({});

        for (int x = 0; x < nX; x++)
        {
            float xPos = static_cast<float>(x) * sizeX + 0.5f * sizeX; // center of voxel

            for (int y = 0; y < nY; y++)
            {
                float v = d(x, y);

                if (drawZeroCells || v > 0)
                {
                    float yPos = (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    gp(0, 3) = xPos;
                    gp(1, 3) = yPos;// we are in mm unit environment -> no conversion to m needed

                    float intensity = static_cast<float>(v);
                    intensity /= maxEntry;

                    if (intensity > 1.0f)
                    {
                        intensity = 1.0f;
                    }

                    if (drawZeroCells || intensity > 0)
                    {
                        std::vector<Eigen::Vector3f> points;
                        points.emplace_back(-sizeX / 2, -sizeY / 2, 0.f);
                        points.emplace_back(-sizeX / 2, sizeY / 2, 0.f);
                        points.emplace_back(sizeX / 2, sizeY / 2, 0.f);
                        points.emplace_back(sizeX / 2, -sizeY / 2, 0.f);

                        auto cube = createPolygon(points);
                        //auto cube = createBox(sizeX, sizeY, 1.f);
                        cube->setColor(cm.getColor(intensity));
                        cube->setGlobalPose(gp);
                        res->addVisualization(cube);

                        if (drawLines)
                        {
                            auto lines = createLineSet(points, 4.f);
                            lines->setColor(Visualization::Color::Black());
                            lines->setGlobalPose(gp);
                            res->addVisualization(lines);
                        }
                    }
                }
            }
        }

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        pose.block<3, 1>(0, 3) = Eigen::Vector3f(-extendCellX * nX / 2, -extendCellY * nY / 2, 0);
        res->setGlobalPose(pose);
        res->setGlobalPoseNoUpdate(Eigen::Matrix4f::Identity()); // move the global pose to center of the returned visualization
        return res;
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
