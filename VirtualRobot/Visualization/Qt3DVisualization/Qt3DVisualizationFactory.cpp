/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualizationFactory.h"
#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"

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

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::string &filename, bool boundingBox) const
    {
        std::cout << "Qt3DFile" << std::endl;
        std::cout << filename << std::endl;
        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        Qt3DRender::QSceneLoader* sceneLoader = new Qt3DRender::QSceneLoader(visu->getEntity());
        visu->getEntity()->addComponent(sceneLoader);
        sceneLoader->setSource(QUrl(QString::fromStdString("file://" + filename)));
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualizationFromFile(const std::ifstream &ifs, bool boundingBox) const
    {
        VR_ERROR_ONCE_NYI;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createVisualisationSet(const std::vector<VisualizationPtr> &visualizations) const
    {
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualizations));
    }

    VisualizationPtr Qt3DVisualizationFactory::createBox(float width, float height, float depth) const
    {
        Qt3DExtras::QCuboidMesh *cuboid = new Qt3DExtras::QCuboidMesh();
        cuboid->setXExtent(width);
        cuboid->setYExtent(height);
        cuboid->setZExtent(depth);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(cuboid);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Vector3f &from, const Eigen::Vector3f &to, float width) const
    {
        auto *geometry = new Qt3DRender::QGeometry();

        QByteArray bufferBytes;
        bufferBytes.resize(3 * 2 * sizeof(float));
        float *positions = reinterpret_cast<float*>(bufferBytes.data());
        *positions++ = from.x();
        *positions++ = from.y();
        *positions++ = from.z();
        *positions++ = to.x();
        *positions++ = to.y();
        *positions++ = to.z();

        auto *buf = new Qt3DRender::QBuffer(geometry);
        buf->setData(bufferBytes);

        auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
        positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
        positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
        positionAttribute->setVertexSize(3);
        positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        positionAttribute->setBuffer(buf);
        positionAttribute->setByteStride(3 * sizeof(float));
        positionAttribute->setCount(2);
        geometry->addAttribute(positionAttribute);

        QByteArray indexBytes;
        indexBytes.resize(2 * sizeof(unsigned int));
        unsigned int *indices = reinterpret_cast<unsigned int*>(indexBytes.data());
        *indices++ = 0;
        *indices++ = 1;

        auto *indexBuffer = new Qt3DRender::QBuffer(geometry);
        indexBuffer->setData(indexBytes);

        auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
        indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedInt);
        indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
        indexAttribute->setBuffer(indexBuffer);
        indexAttribute->setCount(2);
        geometry->addAttribute(indexAttribute);

        auto *line = new Qt3DRender::QGeometryRenderer();
        line->setGeometry(geometry);
        line->setPrimitiveType(Qt3DRender::QGeometryRenderer::Lines);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(line);

        /*auto *material = new Qt3DRender::QMaterial();

        auto *effect = new Qt3DRender::QEffect();
        auto *gl3Technique = new Qt3DRender::QTechnique();
        auto *gl3Pass = new Qt3DRender::QRenderPass();

        auto *lineWidth = new Qt3DRender::QLineWidth();
        lineWidth->setValue(width);
        gl3Pass->addRenderState(lineWidth);

        gl3Technique->addRenderPass(gl3Pass);
        gl3Technique->graphicsApiFilter()->setApi(Qt3DRender::QGraphicsApiFilter::OpenGL);
        gl3Technique->graphicsApiFilter()->setMajorVersion(3);
        gl3Technique->graphicsApiFilter()->setMinorVersion(1);
        gl3Technique->graphicsApiFilter()->setProfile(Qt3DRender::QGraphicsApiFilter::CoreProfile);

        effect->addTechnique(gl3Technique);
        material->setEffect(effect);

        visu->getEntity()->removeComponent(visu->material);
        visu->getEntity()->addComponent(material);*/

        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createLine(const Eigen::Matrix4f &from, const Eigen::Matrix4f &to, float width) const
    {
        Eigen::Vector3f fromVec = from.block<3, 1>(0, 3);
        Eigen::Vector3f toVec = to.block<3, 1>(0, 3);
        return createLine(fromVec, toVec, width);
    }

    VisualizationPtr Qt3DVisualizationFactory::createSphere(float radius) const
    {
        std::cout << "Sphere" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircle(float radius, float circleCompletion, float width, size_t numberOfCircleParts) const
    {
        std::cout << "Circle" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createTorus(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        std::cout << "Torus" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCircleArrow(float radius, float tubeRadius, float completion, int sides, int rings) const
    {
        std::cout << "CircleArrow" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCylinder(float radius, float height) const
    {
        std::cout << "Cylinder" << std::endl;
        Qt3DExtras::QCylinderMesh* cylinder = new Qt3DExtras::QCylinderMesh();
        cylinder->setRadius(radius);
        cylinder->setLength(height);
        cylinder->setSlices(64);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(cylinder);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCoordSystem(std::string *text, float axisLength, float axisSize, int nrOfBlocks) const
    {
        std::cout << "CoordSystem" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createPoint(float radius) const
    {
        std::cout << "Point" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Matrix4f> &points, float radius) const
    {
        std::cout << "PointCloud" << std::endl;
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationSetPtr Qt3DVisualizationFactory::createPointCloud(const std::vector<Eigen::Vector3f> &points, float radius) const
    {
        std::cout << "PointCloud" << std::endl;
        std::vector<VisualizationPtr> visualisations;
        return VisualizationSetPtr(new Qt3DVisualizationSet(visualisations));
    }

    VisualizationPtr Qt3DVisualizationFactory::createTriMeshModel(const TriMeshModelPtr &model) const
    {
        std::cout << "TriMesh" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createArrow(const Eigen::Vector3f &n, float length, float width) const
    {
        std::cout << "Arrow" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const
    {
        std::cout << "Text" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCone(float baseRadius, float height) const
    {
        std::cout << "Cone" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createEllipse(float x, float y, float z) const
    {
        std::cout << "Ellipse" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createContactVisualization(const EndEffector::ContactInfoVector &contacts, float frictionConeHeight, float frictionConeRadius, bool scaleAccordingToApproachDir) const
    {
        std::cout << "Contact" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createVisualization() const
    {
        std::cout << "Empty" << std::endl;
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
