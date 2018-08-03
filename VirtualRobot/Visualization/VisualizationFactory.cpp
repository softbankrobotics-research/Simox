/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @author     Nikolaus Vahrenkamp
* @author     Adrian Knobloch
* @copyright  2010 Manfred Kroehnert
*/


#include "Visualization.h"
#include "VisualizationSet.h"
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
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationSetPtr VisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        return VisualizationSetPtr(new DummyVisualizationSet(visualizations));
    }

    VisualizationPtr VisualizationFactory::createBox(float, float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Vector3f &, const Eigen::Vector3f &, float) const
    {
        return VisualizationPtr(new DummyVisualization);
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
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCircle(float, float, float, size_t) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createTorus(float, float, float, int, int) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCircleArrow(float, float, float, int, int) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCylinder(float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCoordSystem(std::string *text, float axisLength, float axisSize, int nrOfBlocks) const
    {
        std::vector<VisualizationPtr> visus;

        visus.push_back(createArrow(Eigen::Vector3f::UnitX(), axisLength, axisSize));
        visus.back()->setColor(Visualization::Color::Red());
        visus.push_back(createArrow(Eigen::Vector3f::UnitY(), axisLength, axisSize));
        visus.back()->setColor(Visualization::Color::Green());
        visus.push_back(createArrow(Eigen::Vector3f::UnitZ(), axisLength, axisSize));
        visus.back()->setColor(Visualization::Color::Blue());

        if (text)
        {
            visus.push_back(createText(*text, false, 2.f, 2.f, 0.f));
        }

        return createVisualisationSet(visus);
    }

    VisualizationPtr VisualizationFactory::createPoint(float) const
    {
        return VisualizationPtr(new DummyVisualization);
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
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createPolygon(const std::vector<Eigen::Vector3f> &) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createPlane(const Eigen::Vector3f &, const Eigen::Vector3f &, float, const std::string &) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createArrow(const Eigen::Vector3f &, float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createText(const std::string &, bool, float, float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCone(float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createEllipse(float, float, float) const
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &, float, float, bool) const
    {
        //TODO implement using primitives
        return VisualizationPtr(new DummyVisualization);
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
        return VisualizationPtr(new DummyVisualization);
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
