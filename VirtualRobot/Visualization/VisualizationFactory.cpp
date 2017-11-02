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


namespace VirtualRobot
{

    VisualizationFactory::VisualizationFactory()
    {
        ;
    }

    VisualizationFactory::~VisualizationFactory()
    {
        ;
    }

    void VisualizationFactory::init(int &, char *[], const std::string &)
    {
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr> &, bool)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromFile(const std::string &, bool)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createVisualizationFromFile(const std::ifstream &, bool)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationSetPtr VisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        return VisualizationSetPtr(new DummyVisualizationSet(visualizations));
    }

    VisualizationPtr VisualizationFactory::createBox(float, float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Vector3f &, const Eigen::Vector3f &, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createLine(const Eigen::Matrix4f &, const Eigen::Matrix4f &, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f> &from, const std::vector<Eigen::Vector3f> &to, float width)
    {
        VR_ASSERT(from.size() == to.size());
        std::vector<VisualizationPtr> visus;
        for (size_t i = 0; i<from.size(); ++i)
        {
            visus.push_back(createLine(from[i], to[i], width));
        }
        return createVisualisationSet(visus);
    }

    VisualizationSetPtr VisualizationFactory::createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width)
    {
        VR_ASSERT(from.size() == to.size());
        std::vector<VisualizationPtr> visus;
        for (size_t i = 0; i<from.size(); ++i)
        {
            visus.push_back(createLine(from[i], to[i], width));
        }
        return createVisualisationSet(visus);
    }

    VisualizationPtr VisualizationFactory::createSphere(float, float, float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCircle(float, float, float, size_t)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createTorus(float, float, float, int, int)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCircleArrow(float, float, float, int, int)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCylinder(float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createCoordSystem(float, std::string *, const Eigen::Matrix4f &, float, float, int)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createVertexVisualization(const Eigen::Vector3f &, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createTriMeshModelVisualization(TriMeshModelPtr, Eigen::Matrix4f &, bool, bool)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createPlane(const Eigen::Vector3f &, const Eigen::Vector3f &, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createArrow(const Eigen::Vector3f &, float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createText(const std::string &, bool, float, float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createEllipse(float, float, float, bool, float, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createContactVisualization(EndEffector::ContactInfoVector &, float, float, bool)
    {
        //TODO implement using primitives
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createReachabilityVisualization(WorkspaceRepresentationPtr, const ColorMapPtr, bool, float)
    {
        return VisualizationPtr(new DummyVisualization);
    }

    VisualizationPtr VisualizationFactory::createVisualization()
    {
        return VisualizationPtr(new DummyVisualization);
    }

    void VisualizationFactory::cleanup()
    {
        ;
    }

    VisualizationFactoryPtr VisualizationFactory::getGlobalVisualizationFactory()
    {
        return VisualizationFactory::first(NULL);
    }

    std::string VisualizationFactory::getVisualizationType()
    {
        return "dummy";
    }

} // namespace VirtualRobot
