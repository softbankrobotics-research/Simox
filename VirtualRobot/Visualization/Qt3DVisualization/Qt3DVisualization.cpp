/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"

#include <Qt3DRender/QMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QPlaneMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QTorusMesh>

#include <Qt3DExtras/QDiffuseMapMaterial>
#include <Qt3DExtras/QDiffuseSpecularMapMaterial>
#include <Qt3DExtras/QGoochMaterial>
#include <Qt3DExtras/QNormalDiffuseMapMaterial>
#include <Qt3DExtras/QNormalDiffuseMapAlphaMaterial>
#include <Qt3DExtras/QNormalDiffuseSpecularMapMaterial>
#include <Qt3DExtras/QPerVertexColorMaterial>
#include <Qt3DExtras/QPhongAlphaMaterial>
#include <Qt3DExtras/QPhongMaterial>

#include <Qt3DRender/QAbstractLight>
#include <Qt3DRender/QDirectionalLight>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QSpotLight>

#include <Qt3DRender/QBuffer>
#include <Qt3DRender/QBufferDataGenerator>
#include <Qt3DRender/QAttribute>
#include <Qt3DRender/QGeometryFactory>

#include <Qt3DExtras/QTorusMesh>
#include <QPropertyAnimation>

#include "../TriMeshModel.h"
#include "../../VirtualRobotException.h"

namespace VirtualRobot
{
    Qt3DVisualization::Qt3DVisualization()
    {
        this->entity = new Qt3DCore::QEntity();
        this->transformation = new Qt3DCore::QTransform(this->entity);
        this->material = new Qt3DExtras::QPhongMaterial(this->entity);
        this->entity->addComponent(transformation);
        this->entity->addComponent(material);

        this->scaleMatrix = Eigen::Matrix4f::Identity();
        this->globalPose = Eigen::Matrix4f::Identity();
        applyPose();

        this->setColor(Color(0.33f, 0.33f, 0.33f));
        this->setUpdateVisualization(true);
    }

    Qt3DVisualization::~Qt3DVisualization()
    {
        delete this->entity;
    }

    void Qt3DVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Visualization::setGlobalPose(m);
        this->globalPose = m;
        applyPose();
    }

    size_t Qt3DVisualization::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        static unsigned int id = 0;
        poseChangedCallbacks[id] = f;
        return id++;
    }

    void Qt3DVisualization::removePoseChangedCallback(size_t id)
    {
        auto it = poseChangedCallbacks.find(id);
        if (it != poseChangedCallbacks.end())
        {
            poseChangedCallbacks.erase(it);
        }
    }

    void Qt3DVisualization::setVisible(bool showVisualization)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        this->entity->setEnabled(showVisualization);
    }

    bool Qt3DVisualization::isVisible() const
    {
        return this->entity->isEnabled();
    }

    void Qt3DVisualization::setUpdateVisualization(bool enable)
    {
        this->updateVisualization = enable;
    }

    bool Qt3DVisualization::getUpdateVisualizationStatus() const
    {
        return this->updateVisualization;
    }

    void Qt3DVisualization::setStyle(Visualization::DrawStyle s)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        std::cout << "setStyle()" << std::endl;
    }

    Visualization::DrawStyle Qt3DVisualization::getStyle() const
    {
        std::cout << "getStyle()" << std::endl;
    }

    void Qt3DVisualization::setColor(const Visualization::Color &c)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        this->material->setAmbient(QColor((int)(0.2f * c.r * 255.0f), (int)(0.2f * c.g * 255.0f), (int)(0.2f * c.b * 255.0f)));
        this->material->setDiffuse(QColor((int)(c.r * 255.0f), (int)(c.g * 255.0f), (int)(c.b * 255.0f)));
    }

    Visualization::Color Qt3DVisualization::getColor() const
    {
        std::cout << "getColor()" << std::endl;
    }

    void Qt3DVisualization::setMaterial(const Visualization::MaterialPtr &material)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        std::cout << "setMaterial()" << std::endl;
    }

    Visualization::MaterialPtr Qt3DVisualization::getMaterial() const
    {
        std::cout << "getMaterial()" << std::endl;
        return MaterialPtr(new PhongMaterial());
    }

    void Qt3DVisualization::setSelected(bool selected)
    {
        std::cout << "setSelected()" << std::endl;
    }

    bool Qt3DVisualization::isSelected() const
    {
        std::cout << "isSelected()" << std::endl;
    }

    size_t Qt3DVisualization::addSelectionChangedCallback(std::function<void (bool)> f)
    {
        std::cout << "addSelectionChangedCallback()" << std::endl;
    }

    void Qt3DVisualization::removeSelectionChangedCallback(size_t id)
    {
        std::cout << "removeSelectionChangedCallback()" << std::endl;
    }

    void Qt3DVisualization::scale(const Eigen::Vector3f &scaleFactor)
    {
        THROW_VR_EXCEPTION_IF(scaleFactor.x() <= 0 || scaleFactor.y() <= 0 || scaleFactor.z() <= 0, "Scaling must be >0");
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        scaleMatrix(0, 0) *= scaleFactor[0];
        scaleMatrix(1, 1) *= scaleFactor[1];
        scaleMatrix(2, 2) *= scaleFactor[2];
        applyPose();
    }

    void Qt3DVisualization::shrinkFatten(float offset)
    {
        if (!getUpdateVisualizationStatus())
        {
            return;
        }

        std::cout << "shrinkFatten() " << offset << std::endl;
    }

    bool Qt3DVisualization::hasManipulator(Visualization::ManipulatorType t) const
    {
        std::cout << "hasManipulator()" << std::endl;
    }

    std::vector<Visualization::ManipulatorType> Qt3DVisualization::getAddedManipulatorTypes() const
    {
        std::cout << "getAddedManipulatorTypes()" << std::endl;
    }

    std::vector<Primitive::PrimitivePtr> Qt3DVisualization::getPrimitives() const
    {
        std::cout << "getPrimitives()" << std::endl;
    }

    void Qt3DVisualization::setFilename(const std::string &filename, bool boundingBox)
    {
        std::cout << "setFilename()" << std::endl;
    }

    std::string Qt3DVisualization::getFilename() const
    {
        std::cout << "getFilename()" << std::endl;
    }

    bool Qt3DVisualization::usedBoundingBoxVisu() const
    {
        std::cout << "usedBoundingBoxVisu()" << std::endl;
    }

    BoundingBox Qt3DVisualization::getBoundingBox() const
    {
        std::cout << "getBoundingBox()" << std::endl;
    }

    TriMeshModelPtr Qt3DVisualization::getTriMeshModel() const
    {
        TriMeshModelPtr triMesh(new TriMeshModel());
        unsigned int globalIndexCounter = 0;

        for(Qt3DRender::QGeometryRenderer* render : this->entity->findChildren<Qt3DRender::QGeometryRenderer*>())
        {
            if(render->primitiveType() == Qt3DRender::QGeometryRenderer::PrimitiveType::Triangles)
            {
                Qt3DRender::QAttribute *vertexAttribute = nullptr;
                QByteArray vertexData;
                Qt3DRender::QAttribute *normalAttribute = nullptr;
                QByteArray normalData;
                Qt3DRender::QAttribute *indexAttribute = nullptr;
                QByteArray indexData;

                for (Qt3DRender::QAttribute *attribute : render->geometry()->attributes())
                {
                    auto buffer = attribute->buffer();
                    auto generator = attribute->buffer()->dataGenerator();
                    auto name = attribute->name();
                    auto type = attribute->attributeType();

                    if(type == Qt3DRender::QAttribute::VertexAttribute)
                    {
                        if(name.compare(Qt3DRender::QAttribute::defaultPositionAttributeName()) == 0)
                        {
                            if (generator)
                                vertexData = (*generator.data())();
                            else
                                vertexData = buffer->data();
                            vertexAttribute = attribute;
                        }
                        else if(name.compare(Qt3DRender::QAttribute::defaultNormalAttributeName()) == 0)
                        {
                            if (generator)
                                normalData = (*generator.data())();
                            else
                                normalData = buffer->data();
                            normalAttribute = attribute;
                        }
                    }
                    else if(type == Qt3DRender::QAttribute::IndexAttribute)
                    {
                        if (generator)
                            indexData = (*generator.data())();
                        else
                            indexData = buffer->data();
                        indexAttribute = attribute;
                    }
                }

                if(vertexAttribute && normalAttribute && indexAttribute)
                {
                    if(indexAttribute->vertexBaseType() == Qt3DRender::QAttribute::UnsignedShort)
                    {
                        ushort *rawIndexArray = reinterpret_cast<ushort *>(indexData.data());
                        float *rawVertexArray = reinterpret_cast<float *>(vertexData.data());
                        float *rawNormalArray = reinterpret_cast<float *>(normalData.data());

                        for(unsigned int i = vertexAttribute->byteOffset(); i < vertexAttribute->count() * (vertexAttribute->byteStride() / sizeof(float)); i += (vertexAttribute->byteStride() / sizeof(float)))
                        {
                            triMesh->addVertex(MathTools::transformPosition(Eigen::Vector3f(rawVertexArray[i], rawVertexArray[i + 1], rawVertexArray[i + 2]), scaleMatrix));
                        }
                        for(unsigned int i = normalAttribute->byteOffset(); i < normalAttribute->count() * (normalAttribute->byteStride() / sizeof(float)); i += (normalAttribute->byteStride() / sizeof(float)))
                        {
                            //triMesh->addNormal(MathTools::transformPosition(Eigen::Vector3f(rawNormalArray[i], rawNormalArray[i + 1], rawNormalArray[i + 2]), scaleMatrix).normalized());
                            triMesh->addNormal(Eigen::Vector3f(rawNormalArray[i], rawNormalArray[i + 1], rawNormalArray[i + 2]));
                        }
                        for(unsigned int i = 0; i < indexAttribute->count(); i += 3)
                        {
                            triMesh->addFace(globalIndexCounter + rawIndexArray[i], globalIndexCounter + rawIndexArray[i + 1], globalIndexCounter + rawIndexArray[i + 2]);
                        }
                        globalIndexCounter += vertexAttribute->count();
                    }
                    else if(indexAttribute->vertexBaseType() == Qt3DRender::QAttribute::UnsignedInt)
                    {
                        std::cout << "Unsigned Integer Index Buffer detected" << std::endl;
                    }
                }
            }
        }

        return triMesh;
    }

    int Qt3DVisualization::getNumFaces() const
    {
        std::cout << "getNumFaces()" << std::endl;
        return 0;
    }

    VisualizationPtr Qt3DVisualization::clone() const
    {
        Qt3DVisualizationPtr clonedVisu(new Qt3DVisualization());

        for(auto component : this->getEntity()->components())
        {
            auto duplicatedComponent = duplicateComponent(component);
            if(duplicatedComponent)
            {
                clonedVisu->getEntity()->addComponent(duplicatedComponent);

                ComponentTypes type = componentType(duplicatedComponent);
                if(type == Transform)
                {
                    clonedVisu->transformation = dynamic_cast<Qt3DCore::QTransform*>(duplicatedComponent);
                }
                else if(type == MaterialPhong)
                {
                    clonedVisu->material = dynamic_cast<Qt3DExtras::QPhongMaterial*>(duplicatedComponent);
                }
            }
        }

        return clonedVisu;
    }

    void Qt3DVisualization::print() const
    {
        std::cout << "print()" << std::endl;
    }

    std::string Qt3DVisualization::toXML(const std::string &basePath, int tabs) const
    {
        std::cout << "toXML()" << std::endl;
    }

    std::string Qt3DVisualization::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        std::cout << "toXML2()" << std::endl;
    }

    bool Qt3DVisualization::saveModel(const std::string &modelPath, const std::string &filename)
    {
        std::cout << "saveModel()" << std::endl;
    }

    Qt3DCore::QEntity *Qt3DVisualization::getEntity() const
    {
        return this->entity;
    }

    void Qt3DVisualization::createTriMeshModel()
    {
        QList<Qt3DRender::QGeometryRenderer*> renderer = this->entity->findChildren<Qt3DRender::QGeometryRenderer*>();
        std::cout << "Found " << renderer.size() << " renderers" << std::endl;

        for(Qt3DRender::QGeometryRenderer* render : renderer)
        {
            std::cout << "Primitive Type: " << render->primitiveType() << std::endl;
            Qt3DRender::QGeometry* geometry = render->geometry();
            std::cout << "Geometry: " << geometry->objectName().toStdString() << std::endl;
            for (auto attribute : geometry->attributes())
            {
                std::cout << attribute->name().toStdString() << ", size: ";

                auto buffer = attribute->buffer();
                auto generator = attribute->buffer()->dataGenerator();
                QByteArray data;

                if (generator)
                    data = (*generator.data())();
                else
                    data = buffer->data();

                std::cout << data.size() << std::endl;

                /*for(auto i = 0; i < data.size() / sizeof(float); i++)
                {
                    std::cout << *(float*)(&(data.constData()[i * sizeof(float)])) << std::endl;
                }*/
            }
            std::cout << std::endl;
        }
    }

    void Qt3DVisualization::_addManipulator(Visualization::ManipulatorType t)
    {
        std::cout << "_addManipulator()" << std::endl;
    }

    void Qt3DVisualization::_removeManipulator(Visualization::ManipulatorType t)
    {
        std::cout << "_removeManipulator()" << std::endl;
    }

    void Qt3DVisualization::_removeAllManipulators()
    {
        std::cout << "_removeAllManipulators()" << std::endl;
    }

    void Qt3DVisualization::applyPose()
    {
        Eigen::Matrix4f result = globalPose * scaleMatrix;
        this->transformation->setMatrix(QMatrix4x4(result.data()).transposed());
        for (auto& f : poseChangedCallbacks)
        {
            f.second(result);
        }
    }

    Qt3DCore::QComponent *Qt3DVisualization::duplicateComponent(Qt3DCore::QComponent *component) const
    {
        // Check component type and create the same kind
        ComponentTypes type = componentType(component);
        Qt3DCore::QComponent *duplicate = nullptr;

        switch (type) {
        case LightDirectional: {
            Qt3DRender::QDirectionalLight *source =
                    qobject_cast<Qt3DRender::QDirectionalLight *>(component);
            Qt3DRender::QDirectionalLight *newComponent = new Qt3DRender::QDirectionalLight();
            // Copy properties
            newComponent->setColor(source->color());
            newComponent->setWorldDirection(source->worldDirection());
            newComponent->setIntensity(source->intensity());
            duplicate = newComponent;
            break;
        }
        case LightPoint: {
            Qt3DRender::QPointLight *source = qobject_cast<Qt3DRender::QPointLight *>(component);
            Qt3DRender::QPointLight *newComponent = new Qt3DRender::QPointLight();
            newComponent->setColor(source->color());
            newComponent->setConstantAttenuation(source->constantAttenuation());
            newComponent->setIntensity(source->intensity());
            newComponent->setLinearAttenuation(source->linearAttenuation());
            newComponent->setQuadraticAttenuation(source->quadraticAttenuation());
            duplicate = newComponent;
            break;
        }
        case LightSpot: {
            Qt3DRender::QSpotLight *source = qobject_cast<Qt3DRender::QSpotLight *>(component);
            Qt3DRender::QSpotLight *newComponent = new Qt3DRender::QSpotLight();
            newComponent->setColor(source->color());
            newComponent->setConstantAttenuation(source->constantAttenuation());
            newComponent->setCutOffAngle(source->cutOffAngle());
            newComponent->setLocalDirection(source->localDirection());
            newComponent->setIntensity(source->intensity());
            newComponent->setLinearAttenuation(source->linearAttenuation());
            newComponent->setQuadraticAttenuation(source->quadraticAttenuation());
            duplicate = newComponent;
            break;
        }
        case MaterialDiffuseMap: {
            Qt3DExtras::QDiffuseMapMaterial *source =
                    qobject_cast<Qt3DExtras::QDiffuseMapMaterial *>(component);
            Qt3DExtras::QDiffuseMapMaterial *newComponent = new Qt3DExtras::QDiffuseMapMaterial();
            newComponent->setAmbient(source->ambient());
            Qt3DRender::QTextureImage *diffuseTextureImage = new Qt3DRender::QTextureImage();
            diffuseTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                               source->diffuse()->textureImages().at(0))->source());
            newComponent->diffuse()->addTextureImage(diffuseTextureImage);
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            newComponent->setTextureScale(source->textureScale());
            duplicate = newComponent;
            break;
        }
        case MaterialDiffuseSpecularMap: {
            Qt3DExtras::QDiffuseSpecularMapMaterial *source =
                    qobject_cast<Qt3DExtras::QDiffuseSpecularMapMaterial *>(component);
            Qt3DExtras::QDiffuseSpecularMapMaterial *newComponent =
                    new Qt3DExtras::QDiffuseSpecularMapMaterial();
            newComponent->setAmbient(source->ambient());
            Qt3DRender::QTextureImage *diffuseTextureImage = new Qt3DRender::QTextureImage();
            diffuseTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                               source->diffuse()->textureImages().at(0))->source());
            newComponent->diffuse()->addTextureImage(diffuseTextureImage);
            newComponent->setShininess(source->shininess());
            Qt3DRender::QTextureImage *specularTextureImage = new Qt3DRender::QTextureImage();
            specularTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                                source->specular()->textureImages().at(0))->source());
            newComponent->specular()->addTextureImage(specularTextureImage);
            newComponent->setTextureScale(source->textureScale());
            duplicate = newComponent;
            break;
        }
        case MaterialGooch: {
            Qt3DExtras::QGoochMaterial *source = qobject_cast<Qt3DExtras::QGoochMaterial *>(component);
            Qt3DExtras::QGoochMaterial *newComponent = new Qt3DExtras::QGoochMaterial();
            newComponent->setAlpha(source->alpha());
            newComponent->setBeta(source->beta());
            newComponent->setCool(source->cool());
            newComponent->setDiffuse(source->diffuse());
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            newComponent->setWarm(source->warm());
            duplicate = newComponent;
            break;
        }
        case MaterialNormalDiffuseMapAlpha: {
            Qt3DExtras::QNormalDiffuseMapAlphaMaterial *source =
                    qobject_cast<Qt3DExtras::QNormalDiffuseMapAlphaMaterial *>(component);
            Qt3DExtras::QNormalDiffuseMapAlphaMaterial *newComponent =
                    new Qt3DExtras::QNormalDiffuseMapAlphaMaterial();
            newComponent->setAmbient(source->ambient());
            Qt3DRender::QTextureImage *diffuseTextureImage = new Qt3DRender::QTextureImage();
            diffuseTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                               source->diffuse()->textureImages().at(0))->source());
            newComponent->diffuse()->addTextureImage(diffuseTextureImage);
            Qt3DRender::QTextureImage *normalTextureImage = new Qt3DRender::QTextureImage();
            normalTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                              source->normal()->textureImages().at(0))->source());
            newComponent->normal()->addTextureImage(normalTextureImage);
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            newComponent->setTextureScale(source->textureScale());
            duplicate = newComponent;
            break;
        }
        case MaterialNormalDiffuseMap: {
            Qt3DExtras::QNormalDiffuseMapMaterial *source =
                    qobject_cast<Qt3DExtras::QNormalDiffuseMapMaterial *>(component);
            Qt3DExtras::QNormalDiffuseMapMaterial *newComponent =
                    new Qt3DExtras::QNormalDiffuseMapMaterial();
            newComponent->setAmbient(source->ambient());
            Qt3DRender::QTextureImage *diffuseTextureImage = new Qt3DRender::QTextureImage();
            diffuseTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                               source->diffuse()->textureImages().at(0))->source());
            newComponent->diffuse()->addTextureImage(diffuseTextureImage);
            Qt3DRender::QTextureImage *normalTextureImage = new Qt3DRender::QTextureImage();
            normalTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                              source->normal()->textureImages().at(0))->source());
            newComponent->normal()->addTextureImage(normalTextureImage);
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            newComponent->setTextureScale(source->textureScale());
            duplicate = newComponent;
            break;
        }
        case MaterialNormalDiffuseSpecularMap: {
            Qt3DExtras::QNormalDiffuseSpecularMapMaterial *source =
                    qobject_cast<Qt3DExtras::QNormalDiffuseSpecularMapMaterial *>(component);
            Qt3DExtras::QNormalDiffuseSpecularMapMaterial *newComponent =
                    new Qt3DExtras::QNormalDiffuseSpecularMapMaterial();
            newComponent->setAmbient(source->ambient());
            Qt3DRender::QTextureImage *diffuseTextureImage = new Qt3DRender::QTextureImage();
            diffuseTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                               source->diffuse()->textureImages().at(0))->source());
            newComponent->diffuse()->addTextureImage(diffuseTextureImage);
            Qt3DRender::QTextureImage *normalTextureImage = new Qt3DRender::QTextureImage();
            normalTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                              source->normal()->textureImages().at(0))->source());
            newComponent->normal()->addTextureImage(normalTextureImage);
            newComponent->setShininess(source->shininess());
            Qt3DRender::QTextureImage *specularTextureImage = new Qt3DRender::QTextureImage();
            specularTextureImage->setSource(qobject_cast<Qt3DRender::QTextureImage *>(
                                                source->specular()->textureImages().at(0))->source());
            newComponent->specular()->addTextureImage(specularTextureImage);
            newComponent->setTextureScale(source->textureScale());
            duplicate = newComponent;
            break;
        }
        case MaterialPerVertexColor: {
            // MaterialPerVertexColor has no properties
            Qt3DExtras::QPerVertexColorMaterial *newComponent =
                    new Qt3DExtras::QPerVertexColorMaterial();
            duplicate = newComponent;
            break;
        }
        case MaterialPhongAlpha: {
            Qt3DExtras::QPhongAlphaMaterial *source =
                    qobject_cast<Qt3DExtras::QPhongAlphaMaterial *>(component);
            Qt3DExtras::QPhongAlphaMaterial *newComponent = new Qt3DExtras::QPhongAlphaMaterial();
            newComponent->setAlpha(source->alpha());
            newComponent->setAmbient(source->ambient());
            newComponent->setDiffuse(source->diffuse());
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            duplicate = newComponent;
            break;
        }
        case MaterialPhong: {
            Qt3DExtras::QPhongMaterial *source = qobject_cast<Qt3DExtras::QPhongMaterial *>(component);
            Qt3DExtras::QPhongMaterial *newComponent = new Qt3DExtras::QPhongMaterial();
            newComponent->setAmbient(source->ambient());
            newComponent->setDiffuse(source->diffuse());
            newComponent->setShininess(source->shininess());
            newComponent->setSpecular(source->specular());
            duplicate = newComponent;
            break;
        }
        case MeshCuboid: {
            Qt3DExtras::QCuboidMesh *source = qobject_cast<Qt3DExtras::QCuboidMesh *>(component);
            Qt3DExtras::QCuboidMesh *newComponent = new Qt3DExtras::QCuboidMesh();
            newComponent->setXExtent(source->xExtent());
            newComponent->setYExtent(source->yExtent());
            newComponent->setZExtent(source->zExtent());
            newComponent->setXYMeshResolution(source->xyMeshResolution());
            newComponent->setXZMeshResolution(source->xzMeshResolution());
            newComponent->setYZMeshResolution(source->yzMeshResolution());
            duplicate = newComponent;
            break;
        }
        case MeshCustom: {
            Qt3DRender::QMesh *source = qobject_cast<Qt3DRender::QMesh *>(component);
            Qt3DRender::QMesh *newComponent = new Qt3DRender::QMesh();
            newComponent->setSource(source->source());
            duplicate = newComponent;
            break;
        }
        case MeshCylinder: {
            Qt3DExtras::QCylinderMesh *source = qobject_cast<Qt3DExtras::QCylinderMesh *>(component);
            Qt3DExtras::QCylinderMesh *newComponent = new Qt3DExtras::QCylinderMesh();
            newComponent->setLength(source->length());
            newComponent->setRadius(source->radius());
            newComponent->setRings(source->rings());
            newComponent->setSlices(source->slices());
            duplicate = newComponent;
            break;
        }
        case MeshPlane: {
            Qt3DExtras::QPlaneMesh *source = qobject_cast<Qt3DExtras::QPlaneMesh *>(component);
            Qt3DExtras::QPlaneMesh *newComponent = new Qt3DExtras::QPlaneMesh();
            newComponent->setHeight(source->height());
            newComponent->setMeshResolution(source->meshResolution());
            newComponent->setWidth(source->width());
            duplicate = newComponent;
            break;
        }
        case MeshSphere: {
            Qt3DExtras::QSphereMesh *source = qobject_cast<Qt3DExtras::QSphereMesh *>(component);
            Qt3DExtras::QSphereMesh *newComponent = new Qt3DExtras::QSphereMesh();
            newComponent->setGenerateTangents(source->generateTangents());
            newComponent->setRadius(source->radius());
            newComponent->setRings(source->rings());
            newComponent->setSlices(source->slices());
            duplicate = newComponent;
            break;
        }
        case MeshTorus: {
            Qt3DExtras::QTorusMesh *source = qobject_cast<Qt3DExtras::QTorusMesh *>(component);
            Qt3DExtras::QTorusMesh *newComponent = new Qt3DExtras::QTorusMesh();
            newComponent->setMinorRadius(source->minorRadius());
            newComponent->setRadius(source->radius());
            newComponent->setRings(source->rings());
            newComponent->setSlices(source->slices());
            duplicate = newComponent;
            break;
        }
        case MeshGeneric: {
            Qt3DRender::QGeometryRenderer *source = qobject_cast<Qt3DRender::QGeometryRenderer *>(component);
            Qt3DRender::QGeometryRenderer *newComponent = new Qt3DRender::QGeometryRenderer();
            newComponent->setInstanceCount(source->instanceCount());
            newComponent->setVertexCount(source->vertexCount());
            newComponent->setIndexOffset(source->indexOffset());
            newComponent->setFirstInstance(source->firstInstance());
            newComponent->setRestartIndexValue(source->restartIndexValue());
            newComponent->setVerticesPerPatch(source->verticesPerPatch());
            newComponent->setPrimitiveRestartEnabled(source->primitiveRestartEnabled());
            newComponent->setPrimitiveType(source->primitiveType());

            Qt3DRender::QGeometry *sourceGeometry = source->geometry();
            Qt3DRender::QGeometry *newGeometry = new Qt3DRender::QGeometry;
            if (!sourceGeometry) {
                Qt3DRender::QGeometryFactoryPtr geometryFunctorPtr = source->geometryFactory();
                if (geometryFunctorPtr.data())
                    sourceGeometry = geometryFunctorPtr.data()->operator()();
            }
            if (sourceGeometry) {
                QMap<Qt3DRender::QBuffer *, Qt3DRender::QBuffer *> bufferMap;
                Q_FOREACH (Qt3DRender::QAttribute *oldAtt, sourceGeometry->attributes()) {
                    Qt3DRender::QAttribute *newAtt = copyAttribute(oldAtt, bufferMap);
                    if (newAtt)
                        newGeometry->addAttribute(newAtt);
                }

                newGeometry->setBoundingVolumePositionAttribute(
                            copyAttribute(sourceGeometry->boundingVolumePositionAttribute(),
                                          bufferMap));

                newComponent->setGeometry(newGeometry);
            }
            duplicate = newComponent;
            break;
        }
        case Transform: {
            Qt3DCore::QTransform *source = qobject_cast<Qt3DCore::QTransform *>(component);
            Qt3DCore::QTransform *newComponent = new Qt3DCore::QTransform();
            newComponent->setMatrix(source->matrix());
            duplicate = newComponent;
            break;
        }
        case SceneLoader: {
            Qt3DRender::QSceneLoader *source = qobject_cast<Qt3DRender::QSceneLoader *>(component);
            Qt3DRender::QSceneLoader *newComponent = new Qt3DRender::QSceneLoader();
            newComponent->setSource(source->source());
            duplicate = newComponent;
            break;
        }
        case Unknown:
            qWarning() << "Unsupported component:" << component;
            break;
        }

        // Copy property locks, except for transforms
        if (type != Transform)
            copyLockProperties(component, duplicate);

        return duplicate;
    }

    Qt3DVisualization::ComponentTypes Qt3DVisualization::componentType(Qt3DCore::QComponent *component) const
    {
        Qt3DVisualization::ComponentTypes componentType = Unknown;

        if (qobject_cast<Qt3DRender::QAbstractLight *>(component)) {
            if (qobject_cast<Qt3DRender::QDirectionalLight *>(component))
                componentType = LightDirectional;
            else if (qobject_cast<Qt3DRender::QPointLight *>(component))
                componentType = LightPoint;
            else if (qobject_cast<Qt3DRender::QSpotLight *>(component))
                componentType = LightSpot;
        } else if (qobject_cast<Qt3DRender::QMaterial *>(component)) {
            if (qobject_cast<Qt3DExtras::QDiffuseMapMaterial *>(component))
                componentType = MaterialDiffuseMap;
            else if (qobject_cast<Qt3DExtras::QDiffuseSpecularMapMaterial *>(component))
                componentType = MaterialDiffuseSpecularMap;
            else if (qobject_cast<Qt3DExtras::QGoochMaterial *>(component))
                componentType = MaterialGooch;
            // Inherits QNormalDiffuseMapMaterial, so must be tested first
            else if (qobject_cast<Qt3DExtras::QNormalDiffuseMapAlphaMaterial *>(component))
                componentType = MaterialNormalDiffuseMapAlpha;
            else if (qobject_cast<Qt3DExtras::QNormalDiffuseMapMaterial *>(component))
                componentType = MaterialNormalDiffuseMap;
            else if (qobject_cast<Qt3DExtras::QNormalDiffuseSpecularMapMaterial *>(component))
                componentType = MaterialNormalDiffuseSpecularMap;
            else if (qobject_cast<Qt3DExtras::QPerVertexColorMaterial *>(component))
                componentType = MaterialPerVertexColor;
            else if (qobject_cast<Qt3DExtras::QPhongAlphaMaterial *>(component))
                componentType = MaterialPhongAlpha;
            else if (qobject_cast<Qt3DExtras::QPhongMaterial *>(component))
                componentType = MaterialPhong;
            else
                componentType = MaterialGeneric;
        } else if (qobject_cast<Qt3DRender::QGeometryRenderer *>(component)) {
            if (qobject_cast<Qt3DRender::QMesh *>(component))
                componentType = MeshCustom;
            else if (qobject_cast<Qt3DExtras::QCuboidMesh *>(component))
                componentType = MeshCuboid;
            else if (qobject_cast<Qt3DExtras::QCylinderMesh *>(component))
                componentType = MeshCylinder;
            else if (qobject_cast<Qt3DExtras::QPlaneMesh *>(component))
                componentType = MeshPlane;
            else if (qobject_cast<Qt3DExtras::QSphereMesh *>(component))
                componentType = MeshSphere;
            else if (qobject_cast<Qt3DExtras::QTorusMesh *>(component))
                componentType = MeshTorus;
            else
                componentType = MeshGeneric;
        } else if (qobject_cast<Qt3DCore::QTransform *>(component)) {
            componentType = Transform;
        } else if (qobject_cast<Qt3DRender::QSceneLoader *>(component)) {
            componentType = SceneLoader;
        }

        return componentType;
    }

    Qt3DRender::QAttribute *Qt3DVisualization::copyAttribute(
            Qt3DRender::QAttribute *oldAtt,
            QMap<Qt3DRender::QBuffer *, Qt3DRender::QBuffer *> &bufferMap) const
    {
        Qt3DRender::QAttribute *newAtt = nullptr;
        if (oldAtt) {
            newAtt = new Qt3DRender::QAttribute;

            newAtt->setName(oldAtt->name());
            newAtt->setDataType(oldAtt->vertexBaseType());
            newAtt->setDataSize(oldAtt->vertexSize());
            newAtt->setCount(oldAtt->count());
            newAtt->setByteStride(oldAtt->byteStride());
            newAtt->setByteOffset(oldAtt->byteOffset());
            newAtt->setDivisor(oldAtt->divisor());
            newAtt->setAttributeType(oldAtt->attributeType());

            Qt3DRender::QBuffer *oldBuf = oldAtt->buffer();
            if (oldBuf) {
                Qt3DRender::QBuffer *newBuf = bufferMap.value(oldBuf);
                if (!newBuf) {
                    newBuf = new Qt3DRender::QBuffer;
                    bufferMap.insert(oldBuf, newBuf);

                    if (oldBuf->data().isEmpty())
                        newBuf->setData(oldBuf->dataGenerator()->operator()());
                    else
                        newBuf->setData(oldBuf->data());
                    newBuf->setType(oldBuf->type());
                    newBuf->setUsage(oldBuf->usage());
                    newBuf->setSyncData(oldBuf->isSyncData());
                }

                newAtt->setBuffer(newBuf);
            }
        }

        return newAtt;
    }

    void Qt3DVisualization::copyLockProperties(const QObject *source, QObject *target) const
    {
        QList<QByteArray> customProps = source->dynamicPropertyNames();
        Q_FOREACH (const QByteArray &propName, customProps) {
            if (propName.endsWith(lockPropertySuffix8())) {
                target->setProperty(propName.constData(),
                                    source->property(propName.constData()));
            }
        }
    }
}
