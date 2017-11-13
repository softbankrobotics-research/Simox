/**
* @package    VirtualRobot
* @author     Manfred Kroehnert, Adrian Knobloch
* @copyright  2010, 2017 Manfred Kroehnert, Adrian Knobloch
*/


#include "VisualizationSet.h"
#include "Visualization.h"
#include "VisualizationFactory.h"

namespace VirtualRobot
{

    VisualizationSet::VisualizationSet(const std::vector<VisualizationPtr> &visualizations) : Visualization(), visualizations()
    {
        std::copy_if(visualizations.begin(), visualizations.end(), std::back_inserter(this->visualizations), [](const VisualizationPtr& visu) {
            if (visu->isInVisualizationSet())
            {
                VR_WARNING << "Could not add visu to set, because it is already part of a set." << std::endl;
                return false;
            }
            else
            {
                return true;
            }
        });
        if (!this->visualizations.empty())
        {
            for (auto& visu : this->visualizations)
            {
                visu->setSelected(false);
                visu->removeAllManipulators();
                visu->setIsInVisualizationSet(true);
            }
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
            visu->setSelected(this->isSelected());
            visu->removeAllManipulators();
            for (auto& m : this->getAddedManipulatorTypes())
            {
                visu->addManipulator(m);
            }
            visu->setIsInVisualizationSet(true);
            visualizations.push_back(visu);
        }
    }

    bool VisualizationSet::containsVisualization(const VisualizationPtr &visu) const
    {
        return std::find(visualizations.begin(), visualizations.end(), visu) != visualizations.end();
    }

    void VisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {
        auto it = std::find(visualizations.begin(), visualizations.end(), visu);
        if (it != visualizations.end())
        {
            (*it)->setIsInVisualizationSet(false);
            visualizations.erase(it);
        }
    }

    void VisualizationSet::removeVisualization(size_t id)
    {
        this->removeVisualization(this->getVisualizationAt(id));
    }

    std::vector<VisualizationPtr> VisualizationSet::getVisualizations() const
    {
        return visualizations;
    }

    VisualizationPtr VisualizationSet::getVisualizationAt(size_t index) const
    {
        return visualizations.at(index);
    }

    bool VisualizationSet::empty() const
    {
        return visualizations.empty();
    }

    size_t VisualizationSet::size() const
    {
        return visualizations.size();
    }

    Eigen::Matrix4f VisualizationSet::getGlobalPose() const
    {
        Visualization::getGlobalPose();
    }

    Eigen::Matrix4f VisualizationSet::getGlobalPose(size_t id) const
    {
        return this->getVisualizationAt(id)->getGlobalPose();
    }

    void VisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Eigen::Matrix4f oldM = this->getGlobalPose();
        Eigen::Matrix4f dp = m * oldM.inverse();
        Visualization::setGlobalPose(m);
        for (auto& visu : visualizations)
        {
            visu->applyDisplacement(dp);
        }
    }

    void VisualizationSet::setGlobalPose(size_t id, const Eigen::Matrix4f &m)
    {
        this->getVisualizationAt(id)->setGlobalPose(m);
    }

    void VisualizationSet::applyDisplacement(size_t id, const Eigen::Matrix4f &dp)
    {
        this->getVisualizationAt(id)->applyDisplacement(dp);
    }

    void VisualizationSet::setVisible(bool showVisualization)
    {
        for (auto& visu : visualizations)
        {
            visu->setVisible(showVisualization);
        }
    }

    void VisualizationSet::setVisible(size_t id, bool showVisualization)
    {
        this->getVisualizationAt(id)->setVisible(showVisualization);
    }

    bool VisualizationSet::isVisible() const
    {
        for (auto& visu : visualizations)
        {
            if (visu->isVisible())
            {
                return true;
            }
        }
        return false;
    }

    bool VisualizationSet::isVisible(size_t id) const
    {
        return this->getVisualizationAt(id)->isVisible();
    }

    void VisualizationSet::setUpdateVisualization(bool enable)
    {
        for (auto& visu : visualizations)
        {
            visu->setUpdateVisualization(enable);
        }
    }

    void VisualizationSet::setUpdateVisualization(size_t id, bool enable)
    {
        this->getVisualizationAt(id)->setUpdateVisualization(enable);
    }

    bool VisualizationSet::getUpdateVisualizationStatus() const
    {
        for (auto& visu : visualizations)
        {
            if (visu->getUpdateVisualizationStatus())
            {
                return true;
            }
        }
        return false;
    }

    bool VisualizationSet::getUpdateVisualizationStatus(size_t id) const
    {
        return this->getVisualizationAt(id)->getUpdateVisualizationStatus();
    }

    void VisualizationSet::setStyle(Visualization::DrawStyle s)
    {
        for (auto& visu : visualizations)
        {
            visu->setStyle(s);
        }
    }

    void VisualizationSet::setStyle(size_t id, Visualization::DrawStyle s)
    {
        this->getVisualizationAt(id)->setStyle(s);
    }

    Visualization::DrawStyle VisualizationSet::getStyle() const
    {
        if (visualizations.empty())
        {
            return DrawStyle::undefined;
        }
        DrawStyle s = visualizations[0]->getStyle();
        for (auto& visu : visualizations)
        {
            if (s != visu->getStyle())
            {
                return DrawStyle::undefined;
            }
        }
        return s;
    }

    Visualization::DrawStyle VisualizationSet::getStyle(size_t id) const
    {
        return this->getVisualizationAt(id)->getStyle();
    }

    void VisualizationSet::setColor(const Visualization::Color &c)
    {
        for (auto& visu : visualizations)
        {
            visu->setColor(c);
        }
    }

    void VisualizationSet::setColor(size_t id, const Visualization::Color &c)
    {
        this->getVisualizationAt(id)->setColor(c);
    }

    Visualization::Color VisualizationSet::getColor() const
    {
        if (visualizations.empty())
        {
            return Visualization::Color::None();
        }
        Visualization::Color c = visualizations[0]->getColor();
        for (auto& visu : visualizations)
        {
            if (c != visu->getColor())
            {
                return Visualization::Color::None();
            }
        }
        return c;
    }

    Visualization::Color VisualizationSet::getColor(size_t id) const
    {
        return this->getVisualizationAt(id)->getColor();
    }

    void VisualizationSet::setMaterial(const MaterialPtr &material)
    {
        for (auto& visu : visualizations)
        {
            visu->setMaterial(material);
        }
    }

    void VisualizationSet::setMaterial(size_t id, const MaterialPtr &material)
    {
        this->getVisualizationAt(id)->setMaterial(material);
    }

    Visualization::MaterialPtr VisualizationSet::getMaterial() const
    {
        if (visualizations.empty())
        {
            return MaterialPtr(new NoneMaterial);
        }
        MaterialPtr m = visualizations[0]->getMaterial();
        for (auto& visu : visualizations)
        {
            if (m != visu->getMaterial())
            {
                return MaterialPtr(new NoneMaterial);
            }
        }
        return m;
    }

    Visualization::MaterialPtr VisualizationSet::getMaterial(size_t id) const
    {
        return this->getVisualizationAt(id)->getMaterial();
    }

    void VisualizationSet::_setSelected(bool selected)
    {
        for (auto& visu : visualizations)
        {
            visu->_setSelected(selected);
        }
    }

    void VisualizationSet::setScalingFactor(const Eigen::Vector3f &scaleFactor)
    {
        for (auto& visu : visualizations)
        {
            visu->setScalingFactor(scaleFactor);
        }
    }

    void VisualizationSet::setScalingFactor(size_t id, Eigen::Vector3f &scaleFactor)
    {
        this->getVisualizationAt(id)->setScalingFactor(scaleFactor);
    }

    Eigen::Vector3f VisualizationSet::getScalingFactor() const
    {
        if (visualizations.empty())
        {
            return Eigen::Vector3f::Constant(1.f);
        }
        Eigen::Vector3f s = visualizations[0]->getScalingFactor();
        for (auto& visu : visualizations)
        {
            if (s != visu->getScalingFactor())
            {
                return Eigen::Vector3f::Constant(1.f);
            }
        }
        return s;
    }

    Eigen::Vector3f VisualizationSet::getScalingFactor(size_t id) const
    {
        return this->getVisualizationAt(id)->getScalingFactor();
    }

    void VisualizationSet::shrinkFatten(float offset)
    {
        for (auto& visu : visualizations)
        {
            visu->shrinkFatten(offset);
        }
    }

    void VisualizationSet::shrinkFatten(size_t id, float offset)
    {
        this->getVisualizationAt(id)->shrinkFatten(offset);
    }

    void VisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
        for (auto& visu : visualizations)
        {
            visu->_addManipulator(t);
        }
    }

    void VisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        for (auto& visu : visualizations)
        {
            visu->_removeManipulator(t);
        }
    }

    void VisualizationSet::_removeAllManipulators()
    {
        for (auto& visu : visualizations)
        {
            visu->_removeAllManipulators();
        }
    }

    std::vector<Primitive::PrimitivePtr> VisualizationSet::getPrimitives() const
    {
        // TODO
    }

    BoundingBox VisualizationSet::getBoundingBox() const
    {
        BoundingBox b;
        for (auto& visu : visualizations)
        {
            b.addPoints(visu->getBoundingBox().getPoints());
        }
        return b;
    }

    BoundingBox VisualizationSet::getBoundingBox(size_t id) const
    {
        return this->getVisualizationAt(id)->getBoundingBox();
    }

    TriMeshModelPtr VisualizationSet::getTriMeshModel() const
    {
        // TODO
    }

    TriMeshModelPtr VisualizationSet::getTriMeshModel(size_t id) const
    {
        return this->getVisualizationAt(id)->getTriMeshModel();
    }

    void VisualizationSet::createTriMeshModel()
    {
        for (auto& visu : visualizations)
        {
            visu->createTriMeshModel();
        }
    }

    int VisualizationSet::getNumFaces() const
    {
        int n = 0;
        for (auto& visu : visualizations)
        {
            n += visu->getNumFaces();
        }
        return n;
    }

    int VisualizationSet::getNumFaces(size_t id) const
    {
        return this->getVisualizationAt(id)->getNumFaces();
    }

    void VisualizationSet::print(size_t id) const
    {
        return this->getVisualizationAt(id)->print();
    }

    std::string VisualizationSet::toXML(size_t id, const std::string &basePath, int tabs) const
    {
        return this->getVisualizationAt(id)->toXML(basePath, tabs);
    }

    std::string VisualizationSet::toXML(size_t id, const std::string &basePath, const std::string &filename, int tabs) const
    {
        return this->getVisualizationAt(id)->toXML(basePath, filename, tabs);
    }

    bool VisualizationSet::saveModel(size_t id, const std::string &modelPath, const std::string &filename)
    {
        return this->getVisualizationAt(id)->saveModel(modelPath, filename);
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
        auto visu = VisualizationFactory::getGlobalVisualizationFactory()->createVisualisationSet(clonedVisus);
        visu->setVisible(this->isVisible());
        visu->setStyle(this->getStyle());
        visu->setColor(this->getColor());
        visu->setFilename(this->getFilename(), this->usedBoundingBoxVisu());
        visu->setScalingFactor(this->getScalingFactor());
        visu->createTriMeshModel();
        visu->setUpdateVisualization(this->getUpdateVisualizationStatus());
        return visu;
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

    void DummyVisualizationSet::_setSelected(bool selected)
    {
        VisualizationSet::_setSelected(selected);
        selected = selected;
    }

    bool DummyVisualizationSet::isSelected() const
    {
        return selected;
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
        VisualizationSet::_addManipulator(t);
        addedManipulators.insert(t);
    }

    void DummyVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        VisualizationSet::_removeManipulator(t);
        auto pos = addedManipulators.find(t);
        if (pos != addedManipulators.end())
        {
            addedManipulators.erase(pos);
        }
    }

    void DummyVisualizationSet::_removeAllManipulators()
    {
        VisualizationSet::_removeAllManipulators();
        addedManipulators.clear();
    }

    bool DummyVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
        auto pos = addedManipulators.find(t);
        return pos != addedManipulators.end();
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

    void DummyVisualizationSet::print() const
    {
        // TODO
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
