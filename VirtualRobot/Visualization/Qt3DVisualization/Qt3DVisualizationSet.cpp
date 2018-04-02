/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"
#include "Qt3DVisualizationFactory.h"

namespace VirtualRobot
{
    Qt3DVisualizationSet::Qt3DVisualizationSet(const std::vector<VisualizationPtr>& visualizations) : VisualizationSet(visualizations)
    {
        this->entity = new Qt3DCore::QEntity();

        for (auto& visu : this->getVisualizations())
        {
            this->addVisualization(visu);
        }
    }

    Qt3DVisualizationSet::~Qt3DVisualizationSet()
    {
    }

    void Qt3DVisualizationSet::addVisualization(const VisualizationPtr &visu)
    {
        VirtualRobot::Qt3DVisualization* qt3dvisu = dynamic_cast<VirtualRobot::Qt3DVisualization*>(visu.get());
        if(qt3dvisu)
        {
            qt3dvisu->getEntity()->setParent(this->entity);
        }
        else
        {
            VirtualRobot::Qt3DVisualizationSet* qt3dvisu = dynamic_cast<VirtualRobot::Qt3DVisualizationSet*>(visu.get());
            if(qt3dvisu)
            {
                qt3dvisu->getEntity()->setParent(this->entity);
            }
        }
    }

    bool Qt3DVisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {

    }

    void Qt3DVisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationSet::setGlobalPose(m);
        for (auto& visu : this->getVisualizations())
        {
            visu->setGlobalPose(m);
        }
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
        std::cout << "Set: clone()" << std::endl;

        std::vector<VisualizationPtr> clonedVisus;
        const auto& visus = getVisualizations();
        clonedVisus.reserve(visus.size());
        for (const auto& visu : visus)
        {
            clonedVisus.push_back(visu->clone());
        }
        return VisualizationFactory::getInstance()->createVisualisationSet(clonedVisus);
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

    Qt3DCore::QEntity *Qt3DVisualizationSet::getEntity()
    {
        std::cout << "getEntity() =" << std::endl;
        std::cout << this->entity << std::endl;
        return this->entity;
    }
}
