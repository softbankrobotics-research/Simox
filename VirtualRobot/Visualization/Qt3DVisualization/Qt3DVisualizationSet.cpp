/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualizationSet.h"

namespace VirtualRobot
{
    Qt3DVisualizationSet::Qt3DVisualizationSet(const std::vector<VisualizationPtr>& visualizations) : VisualizationSet(visualizations)
    {
    }

    Qt3DVisualizationSet::~Qt3DVisualizationSet()
    {
    }

    void Qt3DVisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Qt3DVisualizationSet::setGlobalPose(m);
    }

    size_t Qt3DVisualizationSet::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
    }

    void Qt3DVisualizationSet::removePoseChangedCallback(size_t id)
    {
    }

    size_t Qt3DVisualizationSet::addSelectionChangedCallback(std::function<void (bool)> f)
    {
    }

    void Qt3DVisualizationSet::removeSelectionChangedCallback(size_t id)
    {
    }

    bool Qt3DVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
    }

    std::vector<Visualization::ManipulatorType> Qt3DVisualizationSet::getAddedManipulatorTypes() const
    {
    }

    void Qt3DVisualizationSet::setFilename(const std::string &filename, bool boundingBox)
    {
    }

    std::string Qt3DVisualizationSet::getFilename() const
    {
    }

    bool Qt3DVisualizationSet::usedBoundingBoxVisu() const
    {
    }

    VisualizationPtr Qt3DVisualizationSet::clone() const
    {
    }

    std::string Qt3DVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
    }

    std::string Qt3DVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
    }

    bool Qt3DVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
    }

    void Qt3DVisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
    }

    void Qt3DVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
    }

    void Qt3DVisualizationSet::_removeAllManipulators()
    {
    }

}