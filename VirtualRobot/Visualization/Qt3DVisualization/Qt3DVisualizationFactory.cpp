/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualizationFactory.h"
#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"
#include "../TriMeshModel.h"

#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QConeMesh>
#include <Qt3DRender/QSceneLoader>
#include <Qt3DExtras/QExtrudedTextMesh>
#include <Qt3DExtras/QPerVertexColorMaterial>

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
        std::vector<Eigen::Vector3f> fromVec;
        fromVec.push_back(from);
        std::vector<Eigen::Vector3f> toVec;
        toVec.push_back(to);
        return createLineSet(fromVec, toVec, width);
    }

    VisualizationSetPtr VirtualRobot::Qt3DVisualizationFactory::createLineSet(const std::vector<Eigen::Vector3f> &from, const std::vector<Eigen::Vector3f> &to, float width) const
    {
        VR_ASSERT(from.size() == to.size());

        auto *geometry = new Qt3DRender::QGeometry();

        auto *vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::BufferType::VertexBuffer, geometry);
        auto *indexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::BufferType::IndexBuffer, geometry);

        QByteArray vertexData;
        vertexData.resize(from.size() * 2 * 3 * sizeof(float));
        QByteArray indexData;
        indexData.resize(from.size() * 2 * sizeof(unsigned int));

        float *positions = reinterpret_cast<float*>(vertexData.data());
        ushort *indices = reinterpret_cast<ushort*>(indexData.data());

        for(int i = 0; i < from.size(); i++)
        {
            *positions++ = from[i].x();
            *positions++ = from[i].y();
            *positions++ = from[i].z();
            *positions++ = to[i].x();
            *positions++ = to[i].y();
            *positions++ = to[i].z();
            *indices++ = (ushort) (i * 2);
            *indices++ = (ushort) (i * 2 + 1);
        }

        vertexBuffer->setData(vertexData);
        indexBuffer->setData(indexData);

        auto *positionAttribute = new Qt3DRender::QAttribute(geometry);
        positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());
        positionAttribute->setVertexBaseType(Qt3DRender::QAttribute::Float);
        positionAttribute->setVertexSize(3);
        positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        positionAttribute->setBuffer(vertexBuffer);
        positionAttribute->setCount(from.size() * 2);
        geometry->addAttribute(positionAttribute);

        auto *indexAttribute = new Qt3DRender::QAttribute(geometry);
        indexAttribute->setVertexBaseType(Qt3DRender::QAttribute::UnsignedShort);
        indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
        indexAttribute->setBuffer(indexBuffer);
        indexAttribute->setCount(from.size() * 2);
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

        return createVisualisationSet({visu});
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
        //Prepare trimesh
        model->addMissingNormals();
        model->addMissingColors();
        //model->smoothNormalSurface();

        Qt3DVisualizationPtr visu(new Qt3DVisualization());

        Qt3DRender::QMaterial *material = new Qt3DExtras::QPerVertexColorMaterial(visu->getEntity());

        Qt3DRender::QGeometryRenderer *customMeshRenderer = new Qt3DRender::QGeometryRenderer(visu->getEntity());
        Qt3DRender::QGeometry *customGeometry = new Qt3DRender::QGeometry(customMeshRenderer);

        Qt3DRender::QBuffer *vertexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, customGeometry);
        Qt3DRender::QBuffer *normalBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, customGeometry);
        Qt3DRender::QBuffer *colorBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::VertexBuffer, customGeometry);
        Qt3DRender::QBuffer *indexBuffer = new Qt3DRender::QBuffer(Qt3DRender::QBuffer::IndexBuffer, customGeometry);

        QByteArray vertexBufferData;
        vertexBufferData.resize(model->faces.size() * 3 * 3 * sizeof(float));

        QByteArray normalBufferData;
        normalBufferData.resize(model->faces.size() * 3 * 3 * sizeof(float));

        QByteArray colorBufferData;
        colorBufferData.resize(model->faces.size() * 3 * 3 * sizeof(float));

        QByteArray indexBufferData;
        indexBufferData.resize(model->faces.size() * 3 * sizeof(ushort));

        float *rawVertexArray = reinterpret_cast<float *>(vertexBufferData.data());
        unsigned int vertex_idx = 0;
        float *rawNormalArray = reinterpret_cast<float *>(normalBufferData.data());
        unsigned int normal_idx = 0;
        float *rawColorArray = reinterpret_cast<float *>(colorBufferData.data());
        unsigned int color_idx = 0;
        ushort *rawIndexArray = reinterpret_cast<ushort *>(indexBufferData.data());
        unsigned int index_idx = 0;

        for(auto face : model->faces)
        {
            for(unsigned int vertexID = 0; vertexID < 3; vertexID++)
            {
                Eigen::Vector3f vertex = model->vertices.at(vertexID == 0 ? face.id1 : (vertexID == 1 ? face.id2 : face.id3));
                Eigen::Vector3f normal = model->normals.at(vertexID == 0 ? face.idNormal1 : (vertexID == 1 ? face.idNormal2 : face.idNormal3));
                Visualization::Color color = model->colors.at(vertexID == 0 ? face.idColor1 : (vertexID == 1 ? face.idColor2 : face.idColor3));

                rawVertexArray[vertex_idx++] = vertex[0];
                rawVertexArray[vertex_idx++] = vertex[1];
                rawVertexArray[vertex_idx++] = vertex[2];
                rawNormalArray[normal_idx++] = normal[0];
                rawNormalArray[normal_idx++] = normal[1];
                rawNormalArray[normal_idx++] = normal[2];
                rawColorArray[color_idx++] = color.r;
                rawColorArray[color_idx++] = color.g;
                rawColorArray[color_idx++] = color.b;

                //Important to split the following two lines of code.
                //Don't move increment into copy operation, it will result in a broken index buffer and therefore have very weird effects!
                rawIndexArray[index_idx] = (ushort) index_idx;
                index_idx++;
            }
        }

        vertexBuffer->setData(vertexBufferData);
        normalBuffer->setData(normalBufferData);
        colorBuffer->setData(colorBufferData);
        indexBuffer->setData(indexBufferData);

        // Attributes
        Qt3DRender::QAttribute *positionAttribute = new Qt3DRender::QAttribute(customGeometry);
        positionAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        positionAttribute->setBuffer(vertexBuffer);
        positionAttribute->setDataType(Qt3DRender::QAttribute::Float);
        positionAttribute->setDataSize(3);
        positionAttribute->setCount(model->faces.size() * 3 * 3);
        positionAttribute->setName(Qt3DRender::QAttribute::defaultPositionAttributeName());

        Qt3DRender::QAttribute *normalAttribute = new Qt3DRender::QAttribute(customGeometry);
        normalAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        normalAttribute->setBuffer(normalBuffer);
        normalAttribute->setDataType(Qt3DRender::QAttribute::Float);
        normalAttribute->setDataSize(3);
        normalAttribute->setCount(model->faces.size() * 3 * 3);
        normalAttribute->setName(Qt3DRender::QAttribute::defaultNormalAttributeName());

        Qt3DRender::QAttribute *colorAttribute = new Qt3DRender::QAttribute(customGeometry);
        colorAttribute->setAttributeType(Qt3DRender::QAttribute::VertexAttribute);
        colorAttribute->setBuffer(colorBuffer);
        colorAttribute->setDataType(Qt3DRender::QAttribute::Float);
        colorAttribute->setDataSize(3);
        colorAttribute->setCount(model->faces.size() * 3 * 3);
        colorAttribute->setName(Qt3DRender::QAttribute::defaultColorAttributeName());

        Qt3DRender::QAttribute *indexAttribute = new Qt3DRender::QAttribute(customGeometry);
        indexAttribute->setAttributeType(Qt3DRender::QAttribute::IndexAttribute);
        indexAttribute->setBuffer(indexBuffer);
        indexAttribute->setDataType(Qt3DRender::QAttribute::UnsignedShort);
        indexAttribute->setDataSize(1);
        indexAttribute->setCount(model->faces.size() * 3);

        customGeometry->addAttribute(positionAttribute);
        customGeometry->addAttribute(normalAttribute);
        customGeometry->addAttribute(colorAttribute);
        customGeometry->addAttribute(indexAttribute);

        customMeshRenderer->setInstanceCount(1);
        customMeshRenderer->setFirstInstance(0);
        customMeshRenderer->setVertexCount(model->faces.size() * 3);
        customMeshRenderer->setFirstVertex(0);

        customMeshRenderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);
        customMeshRenderer->setGeometry(customGeometry);

        visu->getEntity()->addComponent(customMeshRenderer);
        visu->getEntity()->removeComponent(visu->material);
        visu->getEntity()->addComponent(material);

        return visu;
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
        Qt3DVisualizationPtr visu(new Qt3DVisualization());

        auto *textMesh = new Qt3DExtras::QExtrudedTextMesh(visu->getEntity());
        textMesh->setDepth(.65f);
        textMesh->setText(QString::fromStdString(text));

        Eigen::Matrix4f textOffset = Eigen::Matrix4f::Identity();
        textOffset(0, 3) = offsetX;
        textOffset(1, 3) = offsetY;
        textOffset(2, 3) = offsetZ;
        visu->setGlobalPose(textOffset);
        visu->scale(10.0f);

        visu->getEntity()->addComponent(textMesh);
        return createVisualisationSet({visu});
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
