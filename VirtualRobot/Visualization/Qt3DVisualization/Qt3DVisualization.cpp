/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"

#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <QPropertyAnimation>

#include "../TriMeshModel.h"

namespace VirtualRobot
{
    Qt3DVisualization::Qt3DVisualization()
    {
        this->entity = new Qt3DCore::QEntity();
        this->transformation = new Qt3DCore::QTransform;
        this->material = new Qt3DExtras::QPhongMaterial(this->entity);

        this->entity->addComponent(transformation);
        this->entity->addComponent(material);
    }

    Qt3DVisualization::~Qt3DVisualization()
    {

    }

    void Qt3DVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        std::cout << "setGlobalPose()" << std::endl;
        Visualization::setGlobalPose(m);
        this->transformation->setMatrix(QMatrix4x4(m.data()));
    }

    size_t Qt3DVisualization::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        std::cout << "addPoseChangedCallback()" << std::endl;
    }

    void Qt3DVisualization::removePoseChangedCallback(size_t id)
    {
        std::cout << "removePoseChangedCallback()" << std::endl;
    }

    void Qt3DVisualization::setVisible(bool showVisualization)
    {
        std::cout << "setVisible()" << std::endl;
    }

    bool Qt3DVisualization::isVisible() const
    {
        std::cout << "isVisible()" << std::endl;
    }

    void Qt3DVisualization::setUpdateVisualization(bool enable)
    {
        std::cout << "setUpdateVisualization()" << std::endl;
    }

    bool Qt3DVisualization::getUpdateVisualizationStatus() const
    {
        std::cout << "getUpdateVisualizationStatus()" << std::endl;
    }

    void Qt3DVisualization::setStyle(Visualization::DrawStyle s)
    {
        std::cout << "setStyle()" << std::endl;
    }

    Visualization::DrawStyle Qt3DVisualization::getStyle() const
    {
        std::cout << "getStyle()" << std::endl;
    }

    void Qt3DVisualization::setColor(const Visualization::Color &c)
    {
        std::cout << "setColor()" << std::endl;
    }

    Visualization::Color Qt3DVisualization::getColor() const
    {
        std::cout << "getColor()" << std::endl;
    }

    void Qt3DVisualization::setMaterial(const Visualization::MaterialPtr &material)
    {
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
        std::cout << "scale()" << std::endl;
    }

    void Qt3DVisualization::shrinkFatten(float offset)
    {
        std::cout << "shrinkFatten()" << std::endl;
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
        std::cout << "getTriMeshModel()" << std::endl;
        return TriMeshModelPtr(new TriMeshModel());
    }

    int Qt3DVisualization::getNumFaces() const
    {
        std::cout << "getNumFaces()" << std::endl;
    }

    VisualizationPtr Qt3DVisualization::clone() const
    {
        std::cout << "clone()" << std::endl;
        return VisualizationPtr(new Qt3DVisualization());
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
        std::cout << "getEntity() =" << std::endl;
        std::cout << this->entity << std::endl;
        return this->entity;
    }

    void Qt3DVisualization::createTriMeshModel()
    {
        std::cout << "createTriMeshModel()" << std::endl;
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
}
