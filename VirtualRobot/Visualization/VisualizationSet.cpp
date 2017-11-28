/**
* @package    VirtualRobot
* @author     Manfred Kroehnert, Adrian Knobloch
* @copyright  2010, 2017 Manfred Kroehnert, Adrian Knobloch
*/


#include "VisualizationSet.h"
#include "Visualization.h"
#include "VisualizationFactory.h"
#include "TriMeshModel.h"

namespace VirtualRobot
{

    VisualizationSet::VisualizationSet(const std::vector<VisualizationPtr> &visualizations)
        : VisualizationGroup(),
          Visualization()
    {
        for (auto& visu : visualizations)
        {
            addVisualization(visu);
        }
    }

    VisualizationSet::~VisualizationSet()
    {
        for (auto& visu : visualizations)
        {
            visu->setIsInVisualizationSet(false);
        }
    }

    void VisualizationSet::addVisualization(const VisualizationPtr &visu)
    {
        if (visu->isInVisualizationSet())
        {
            VR_WARNING << "Could not add visu to set, because it is already part of a set." << std::endl;
        }
        else
        {
            VisualizationGroup::addVisualization(visu);
            visu->setIsInVisualizationSet(true);
        }
    }

    bool VisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {
        if (VisualizationGroup::removeVisualization(visu))
        {
            visu->setIsInVisualizationSet(false);
            return true;
        }
        return false;
    }

    Eigen::Matrix4f VisualizationSet::getGlobalPose() const
    {
        return VisualizationGroup::getGlobalPose();
    }

    void VisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationGroup::setGlobalPose(m);
    }

    void VisualizationSet::applyDisplacement(const Eigen::Matrix4f &dp)
    {
        VisualizationGroup::applyDisplacement(dp);
    }

    void VisualizationSet::setVisible(bool showVisualization)
    {
        VisualizationGroup::setVisible(showVisualization);
    }

    bool VisualizationSet::isVisible() const
    {
        return VisualizationGroup::isVisible();
    }

    void VisualizationSet::setUpdateVisualization(bool enable)
    {
        VisualizationGroup::setUpdateVisualization(enable);
    }

    bool VisualizationSet::getUpdateVisualizationStatus() const
    {
        return VisualizationGroup::getUpdateVisualizationStatus();
    }

    void VisualizationSet::setStyle(Visualization::DrawStyle s)
    {
        VisualizationGroup::setStyle(s);
    }

    Visualization::DrawStyle VisualizationSet::getStyle() const
    {
        return VisualizationGroup::getStyle();
    }

    void VisualizationSet::setColor(const Visualization::Color &c)
    {
        VisualizationGroup::setColor(c);
    }

    Visualization::Color VisualizationSet::getColor() const
    {
        return VisualizationGroup::getColor();
    }

    void VisualizationSet::setMaterial(const MaterialPtr &material)
    {
        VisualizationGroup::setMaterial(material);
    }

    Visualization::MaterialPtr VisualizationSet::getMaterial() const
    {
        return VisualizationGroup::getMaterial();
    }

    void VisualizationSet::setSelected(bool selected)
    {
        for (auto& visu : visualizations)
        {
            visu->setSelected(selected);
        }
    }

    bool VisualizationSet::isSelected() const
    {
        for (auto& visu : visualizations)
        {
            if (!visu->isSelected())
            {
                return false;
            }
        }
        return true;
    }

    void VisualizationSet::scale(const Eigen::Vector3f &s)
    {
        VisualizationGroup::scale(s);
    }

    void VisualizationSet::shrinkFatten(float offset)
    {
        for (auto& visu : visualizations)
        {
            visu->shrinkFatten(offset);
        }
    }

    std::vector<Primitive::PrimitivePtr> VisualizationSet::getPrimitives() const
    {
        std::vector<Primitive::PrimitivePtr> ret;
        for (const auto& visu : visualizations)
        {
            auto p = visu->getPrimitives();
            ret.insert(ret.end(), p.begin(), p.end());
        }
        return ret;
    }

    BoundingBox VisualizationSet::getBoundingBox() const
    {
        return VisualizationGroup::getBoundingBox();
    }

    int VisualizationSet::getNumFaces() const
    {
        return VisualizationGroup::getNumFaces();
    }

    void VisualizationSet::print() const
    {
        VisualizationGroup::print();
    }

    void VisualizationSet::createTriMeshModel()
    {
        for (auto& visu : visualizations)
        {
            visu->createTriMeshModel();
        }
    }

    DummyVisualizationSet::DummyVisualizationSet(const std::vector<VisualizationPtr> &visualizations)
        : VisualizationSet(visualizations),
          selected(false),
          filename(""),
          usedBoundingBox(false)
    {
    }

    VisualizationPtr DummyVisualizationSet::clone() const
    {
        std::vector<VisualizationPtr> clonedVisus;
        clonedVisus.reserve(visualizations.size());
        for (auto& visu : visualizations)
        {
            clonedVisus.push_back(visu->clone());
        }
        return VisualizationFactory::getGlobalVisualizationFactory()->createVisualisationSet(clonedVisus);
    }

    void DummyVisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationSet::setGlobalPose(m);
        for (auto& f : poseChangedCallbacks)
        {
            f.second(m);
        }
    }

    size_t DummyVisualizationSet::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        static size_t id = 0;
        poseChangedCallbacks[id] = f;
        return id++;
    }

    void DummyVisualizationSet::removePoseChangedCallback(size_t id)
    {
        auto it = poseChangedCallbacks.find(id);
        if (it != poseChangedCallbacks.end())
        {
            poseChangedCallbacks.erase(it);
        }
    }

    size_t DummyVisualizationSet::addSelectionChangedCallback(std::function<void (bool)> f)
    {
        VR_ERROR << "NYI" << std::endl;
        return 0;
    }

    void DummyVisualizationSet::removeSelectionChangedCallback(size_t id)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void DummyVisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
        addedManipulators.insert(t);
    }

    void DummyVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        auto pos = addedManipulators.find(t);
        if (pos != addedManipulators.end())
        {
            addedManipulators.erase(pos);
        }
    }

    void DummyVisualizationSet::_removeAllManipulators()
    {
        addedManipulators.clear();
    }

    bool DummyVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
        return addedManipulators.find(t) != addedManipulators.end();
    }

    std::vector<Visualization::ManipulatorType> DummyVisualizationSet::getAddedManipulatorTypes() const
    {
        return std::vector<ManipulatorType>(addedManipulators.begin(), addedManipulators.end());
    }

    void DummyVisualizationSet::setFilename(const std::string &filename, bool boundingBox)
    {
        this->filename = filename;
        usedBoundingBox = boundingBox;
    }

    std::string DummyVisualizationSet::getFilename() const
    {
        return filename;
    }

    bool DummyVisualizationSet::usedBoundingBoxVisu() const
    {
        return usedBoundingBox;
    }

    TriMeshModelPtr DummyVisualizationSet::getTriMeshModel() const
    {
        return TriMeshModelPtr(new TriMeshModel);
    }

    std::string DummyVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        // TODO
        return "";
    }

    std::string DummyVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        // TODO
        return "";
    }

    bool DummyVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        // TODO
        return false;
    }

} // namespace VirtualRobot
