/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualizationFactory.h"
#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"
#include "Qt3DSelectionGroup.h"

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DRender/QSceneLoader>

namespace VirtualRobot
{
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
        std::cout << "Primitives" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::string &filename, bool boundingBox) const
    {
        std::cout << "Qt3DFile" << std::endl;
        std::cout << filename << std::endl;
        Qt3DVisualizationPtr visu = createQt3DVisualization();
        Qt3DRender::QSceneLoader* sceneLoader = new Qt3DRender::QSceneLoader(visu->getEntity());
        visu->getEntity()->addComponent(sceneLoader);
        sceneLoader->setSource(QUrl(QString::fromStdString("file://" + filename)));
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const
    {
        std::cout << "FileStream" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        VisualizationSetPtr set(new Qt3DVisualizationSet(visualizations));
        set->init();
        return set;
    }

    SelectionGroupPtr Qt3DVisualizationFactory::createSelectionGroup() const
    {
        return SelectionGroupPtr(new Qt3DSelectionGroup());
    }

    VisualizationPtr Qt3DVisualizationFactory::createBox(float width, float height, float depth) const
    {
        std::cout << "Box" << std::endl;
        Qt3DExtras::QCuboidMesh *cuboid = new Qt3DExtras::QCuboidMesh();
        cuboid->setXExtent(width);
        cuboid->setYExtent(height);
        cuboid->setZExtent(depth);

        auto visu = createQt3DVisualization();
        visu->getEntity()->addComponent(cuboid);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const
    {
        std::cout << "Line" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width) const
    {
        std::cout << "Line" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f> &from, const std::vector<Eigen::Vector3f> &to, float width) const
    {
        std::cout << "LineSet" << std::endl;
        return VisualizationFactory::createLineSet(from, to, width);
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createLineSet(const std::vector<Eigen::Matrix4f> &from, const std::vector<Eigen::Matrix4f> &to, float width) const
    {
        std::cout << "LineSet" << std::endl;
        return VisualizationFactory::createLineSet(from, to, width);
    }

    VisualizationPtr Qt3DVisualizationFactory::createSphere(float radius) const
    {
        std::cout << "Sphere" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const
    {
        std::cout << "Circle" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createTorus(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        std::cout << "Torus" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        std::cout << "CircleArrow" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCylinder(float radius, float height) const
    {
        std::cout << "Cylinder" << std::endl;
        Qt3DExtras::QCylinderMesh* cylinder = new Qt3DExtras::QCylinderMesh();
        cylinder->setRadius(radius);
        cylinder->setLength(height);
        cylinder->setSlices(64);

        auto visu = createQt3DVisualization();
        visu->getEntity()->addComponent(cylinder);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createPoint(float radius) const
    {
        std::cout << "Point" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Matrix4f> &points, float radius) const
    {
        std::cout << "PointCloud" << std::endl;
        return VisualizationFactory::createPointCloud(points, radius);
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Vector3f> &points, float radius) const
    {
        std::cout << "PointCloud" << std::endl;
        return VisualizationFactory::createPointCloud(points, radius);
    }

    VisualizationPtr Qt3DVisualizationFactory::createTriMeshModel(const TriMeshModelPtr &model) const
    {
        std::cout << "TriMesh" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createArrow(const Eigen::Vector3f &n, float length, float width) const
    {
        std::cout << "Arrow" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const
    {
        std::cout << "Text" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCone(float baseRadius, float height) const
    {
        std::cout << "Cone" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createEllipse(float x, float y, float z) const
    {
        std::cout << "Ellipse" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const
    {
        std::cout << "Contact" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualization() const
    {
        std::cout << "Empty" << std::endl;
        auto visu = createQt3DVisualization();
        return visu;
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

    std::shared_ptr<Qt3DVisualization> Qt3DVisualizationFactory::createQt3DVisualization()
    {
        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->init();
        return visu;
    }
}
