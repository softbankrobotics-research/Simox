/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <QPropertyAnimation>

namespace VirtualRobot
{
    Qt3DVisualization::Qt3DVisualization()
    {
        this->entity = new Qt3DCore::QEntity();
        this->transformation = new Qt3DCore::QTransform;
        this->material = new Qt3DExtras::QPhongMaterial(this->entity);

        /*//EXEMPLARY CONTENT/////////////////////////////////////////////////////
        Qt3DExtras::QTorusMesh* mesh = new Qt3DExtras::QTorusMesh;
        mesh->setRadius(5);
        mesh->setMinorRadius(1);
        mesh->setRings(100);
        mesh->setSlices(20);
        this->entity->addComponent(mesh);
        //this->transformation->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 45.f));
        QPropertyAnimation *rotateTransformAnimation = new QPropertyAnimation(this->transformation);
        rotateTransformAnimation->setTargetObject(this->transformation);
        rotateTransformAnimation->setPropertyName("rotationX");
        rotateTransformAnimation->setStartValue(QVariant::fromValue(0));
        rotateTransformAnimation->setEndValue(QVariant::fromValue(360));
        rotateTransformAnimation->setDuration(10000);
        rotateTransformAnimation->setLoopCount(-1);
        rotateTransformAnimation->start();
        ///////////////////////////////////////////////////////////////////////*/


        this->entity->addComponent(transformation);
        this->entity->addComponent(material);
    }

    Qt3DVisualization::~Qt3DVisualization()
    {

    }

    void Qt3DVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Visualization::setGlobalPose(m);
        this->transformation->setMatrix(QMatrix4x4(m.data()));
        this->transformation->setTranslation(QVector3D(*(m.data() + 3), *(m.data() + 7), *(m.data() + 11)));
    }

    size_t Qt3DVisualization::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
    }

    void Qt3DVisualization::removePoseChangedCallback(size_t id)
    {
    }

    void Qt3DVisualization::setVisible(bool showVisualization)
    {
    }

    bool Qt3DVisualization::isVisible() const
    {
    }

    void Qt3DVisualization::setUpdateVisualization(bool enable)
    {
    }

    bool Qt3DVisualization::getUpdateVisualizationStatus() const
    {
    }

    void Qt3DVisualization::setStyle(Visualization::DrawStyle s)
    {
    }

    Visualization::DrawStyle Qt3DVisualization::getStyle() const
    {
    }

    void Qt3DVisualization::setColor(const Visualization::Color &c)
    {
    }

    Visualization::Color Qt3DVisualization::getColor() const
    {
    }

    void Qt3DVisualization::setMaterial(const Visualization::MaterialPtr &material)
    {
    }

    Visualization::MaterialPtr Qt3DVisualization::getMaterial() const
    {
    }

    void Qt3DVisualization::setSelected(bool selected)
    {
    }

    bool Qt3DVisualization::isSelected() const
    {
    }

    size_t Qt3DVisualization::addSelectionChangedCallback(std::function<void (bool)> f)
    {
    }

    void Qt3DVisualization::removeSelectionChangedCallback(size_t id)
    {
    }

    void Qt3DVisualization::scale(const Eigen::Vector3f &scaleFactor)
    {
    }

    void Qt3DVisualization::shrinkFatten(float offset)
    {
    }

    bool Qt3DVisualization::hasManipulator(Visualization::ManipulatorType t) const
    {
    }

    std::vector<Visualization::ManipulatorType> Qt3DVisualization::getAddedManipulatorTypes() const
    {
    }

    std::vector<Primitive::PrimitivePtr> Qt3DVisualization::getPrimitives() const
    {
    }

    void Qt3DVisualization::setFilename(const std::string &filename, bool boundingBox)
    {
    }

    std::string Qt3DVisualization::getFilename() const
    {
    }

    bool Qt3DVisualization::usedBoundingBoxVisu() const
    {
    }

    BoundingBox Qt3DVisualization::getBoundingBox() const
    {
    }

    TriMeshModelPtr Qt3DVisualization::getTriMeshModel() const
    {
    }

    int Qt3DVisualization::getNumFaces() const
    {
    }

    VisualizationPtr Qt3DVisualization::clone() const
    {
    }

    void Qt3DVisualization::print() const
    {
    }

    std::string Qt3DVisualization::toXML(const std::string &basePath, int tabs) const
    {
    }

    std::string Qt3DVisualization::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
    }

    bool Qt3DVisualization::saveModel(const std::string &modelPath, const std::string &filename)
    {
    }

    Qt3DCore::QEntity *Qt3DVisualization::getEntity()
    {
        return this->entity;
    }

    void Qt3DVisualization::createTriMeshModel()
    {
    }

    void Qt3DVisualization::_addManipulator(Visualization::ManipulatorType t)
    {
    }

    void Qt3DVisualization::_removeManipulator(Visualization::ManipulatorType t)
    {
    }

    void Qt3DVisualization::_removeAllManipulators()
    {
    }

}
