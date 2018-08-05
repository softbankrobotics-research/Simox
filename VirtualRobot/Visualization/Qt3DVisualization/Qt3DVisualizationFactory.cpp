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
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QConeMesh>
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

        auto *buf = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::BufferType::VertexBuffer, geometry);
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

        auto *indexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::BufferType::IndexBuffer, geometry);
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

    VisualizationPtr Qt3DVisualizationFactory::createSphere(float radius) const
    {
        Qt3DExtras::QSphereMesh *sphere = new Qt3DExtras::QSphereMesh();
        sphere->setRadius(radius);
        sphere->setRings(64);
        sphere->setSlices(64);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(sphere);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createCylinder(float radius, float height) const
    {
        Qt3DExtras::QCylinderMesh* cylinder = new Qt3DExtras::QCylinderMesh();
        cylinder->setRadius(radius);
        cylinder->setLength(height);
        cylinder->setSlices(64);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(cylinder);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createPoint(float radius) const
    {
        Qt3DExtras::QSphereMesh *sphere = new Qt3DExtras::QSphereMesh();
        sphere->setRadius(radius);
        sphere->setRings(2);
        sphere->setSlices(4);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(sphere);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createTriMeshModel(const TriMeshModelPtr &model) const
    {
        std::cout << "TriMesh" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createArrow(const Eigen::Vector3f &n, float length, float width) const
    {
        Eigen::Vector3f n2 = n;
        if (n2.norm()<1e-10)
            n2 << 0,0,1;
        n2.normalize();
        float coneHeight = width * 6.0f;
        float coneBottomRadius = width * 2.5f;
        float baseLength = length - coneHeight;
        baseLength = std::max(0.0f, baseLength);
        Eigen::Matrix4f rotation = MathTools::quat2eigen4f(MathTools::getRotation(Eigen::Vector3f::UnitY(), n));
        Eigen::Matrix4f cylinderPosition = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f conePosition = Eigen::Matrix4f::Identity();
        cylinderPosition(1, 3) = baseLength * 0.5f;
        conePosition(1, 3) = baseLength * 0.5f + length * 0.5f;

        auto cylinder = createCylinder(width, baseLength);
        cylinder->setGlobalPose(rotation * cylinderPosition);

        auto cone = createCone(coneBottomRadius, coneHeight);
        cone->setGlobalPose(rotation * conePosition);

        return createVisualisationSet({cylinder, cone});
    }

    VisualizationPtr Qt3DVisualizationFactory::createText(const std::string &text, bool billboard, float offsetX, float offsetY, float offsetZ) const
    {
        std::cout << "Text" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
    }

    VisualizationPtr Qt3DVisualizationFactory::createCone(float baseRadius, float height) const
    {
        Qt3DExtras::QConeMesh* cone = new Qt3DExtras::QConeMesh();
        cone->setBottomRadius(baseRadius);
        cone->setLength(height);

        Qt3DVisualizationPtr visu(new Qt3DVisualization());
        visu->getEntity()->addComponent(cone);
        return visu;
    }

    VisualizationPtr Qt3DVisualizationFactory::createEllipse(float x, float y, float z) const
    {
        // check for min size
        float minSize = 1e-6f;

        if (x < minSize)
        {
            x = minSize;
        }

        if (y < minSize)
        {
            y = minSize;
        }

        if (z < minSize)
        {
            z = minSize;
        }

        VisualizationPtr visu = createSphere(1.0f);
        visu->scale(Eigen::Vector3f(x, y, z));
        return visu;
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
