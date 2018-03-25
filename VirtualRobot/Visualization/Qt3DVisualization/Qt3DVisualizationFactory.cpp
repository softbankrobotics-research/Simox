/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualizationFactory.h"
#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"

namespace VirtualRobot
{
    /**
    * register this class in the super class factory
    */
    VirtualRobot::VisualizationFactory::SubClassRegistry VirtualRobot::Qt3DVisualizationFactory::registry(VirtualRobot::Qt3DVisualizationFactory::getName(),
                                                                                                          &VirtualRobot::Qt3DVisualizationFactory::createInstance);

    Qt3DVisualizationFactory::Qt3DVisualizationFactory()
    {
    }

    Qt3DVisualizationFactory::~Qt3DVisualizationFactory()
    {
    }

    void Qt3DVisualizationFactory::init(int &argc, char *argv[], const std::string &appName)
    {
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr> &primitives, bool boundingBox) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::string &filename, bool boundingBox) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualizations));
    }

    VisualizationPtr Qt3DVisualizationFactory::createBox(float width, float height, float depth) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f> &from, const std::vector<Eigen::Vector3f> &to, float width) const
    {
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width) const
    {
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationPtr Qt3DVisualizationFactory::createSphere(float radius) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createTorus(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCylinder(float radius, float height) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCoordSystem(std::string *text, float axisLength, float axisSize, int nrOfBlocks) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createPoint(float radius) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Matrix4f> &points, float radius) const
    {
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Vector3f> &points, float radius) const
    {
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationPtr Qt3DVisualizationFactory::createTriMeshModel(const TriMeshModelPtr &model) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createArrow(const Eigen::Vector3f &n, float length, float width) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCone(float baseRadius, float height) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createEllipse(float x, float y, float z) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualization() const
    {
        return VisualizationPtr(new Qt3DVisualization());
    }

    void Qt3DVisualizationFactory::cleanup()
    {
    }

    std::string Qt3DVisualizationFactory::getVisualizationType() const
    {
        return getName();
    }

    std::string Qt3DVisualizationFactory::getName()
    {
        return "qt3d";
    }
}
