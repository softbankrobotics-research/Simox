/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"

namespace VirtualRobot
{

    Qt3DVisualization::Qt3DVisualization()
    {

    }

    Qt3DVisualization::~Qt3DVisualization()
    {

    }

    void Qt3DVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Visualization::setGlobalPose(m);
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
