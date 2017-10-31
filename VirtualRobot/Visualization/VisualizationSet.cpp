/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
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

    VisualizationPtr VisualizationSet::clone(float scaling) const
    {
        std::vector<VisualizationPtr> clonedVisus;
        clonedVisus.reserve(visualizations.size());
        for (auto& visu : visualizations)
        {
            clonedVisus.push_back(visu->clone(1.f));
        }
        auto visu = VisualizationFactory::getGlobalVisualizationFactory()->createVisualisationSet(clonedVisus);
        visu->setVisible(this->isVisible());
        visu->setStyle(this->getStyle());
        visu->setColor(this->getColor());
        visu->setFilename(this->getFilename(), this->usedBoundingBoxVisu());
        visu->setScalingFactor(this->getScalingFactor());
        visu->scale(scaling);
        visu->createTriMeshModel();
        visu->setUpdateVisualization(this->getUpdateVisualizationStatus());
        return visu;
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

    Visualization::Color VisualizationSet::getColor(size_t id) const
    {
        return this->getVisualizationAt(id)->getColor();
    }

    void VisualizationSet::setMaterial(const Visualization::Material &material)
    {
        for (auto& visu : visualizations)
        {
            visu->setMaterial(material);
        }
    }

    void VisualizationSet::setMaterial(size_t id, const Visualization::Material &material)
    {
        this->getVisualizationAt(id)->setMaterial(material);
    }

    Visualization::Material VisualizationSet::getMaterial(size_t id) const
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

    void VisualizationSet::print() const
    {
        // TODO
    }

    void VisualizationSet::print(size_t id) const
    {
        return this->getVisualizationAt(id)->print();
    }

    std::string VisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        // TODO
    }

    std::string VisualizationSet::toXML(size_t id, const std::string &basePath, int tabs) const
    {
        return this->getVisualizationAt(id)->toXML(basePath, tabs);
    }

    std::string VisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        // TODO
    }

    std::string VisualizationSet::toXML(size_t id, const std::string &basePath, const std::string &filename, int tabs) const
    {
        return this->getVisualizationAt(id)->toXML(basePath, filename, tabs);
    }

    bool VisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        // TODO
    }

    bool VisualizationSet::saveModel(size_t id, const std::string &modelPath, const std::string &filename)
    {
        return this->getVisualizationAt(id)->saveModel(modelPath, filename);
    }

    DummyVisualizationSet::DummyVisualizationSet(const std::vector<VisualizationPtr> &visualizations)
        : VisualizationSet(visualizations)
    {
    }

    VisualizationPtr DummyVisualizationSet::clone(float scaling) const
    {
        return VisualizationSet::clone(scaling);
    }

    void DummyVisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationSet::setGlobalPose(m);
    }

    void DummyVisualizationSet::applyDisplacement(const Eigen::Matrix4f &dp)
    {
        DummyVisualization::applyDisplacement(dp);
        VisualizationSet::applyDisplacement(dp);
    }

    void DummyVisualizationSet::setVisible(bool showVisualization)
    {
        DummyVisualization::setVisible(showVisualization);
        VisualizationSet::setVisible(showVisualization);
    }

    bool DummyVisualizationSet::isVisible() const
    {
        return DummyVisualization::isVisible();
    }

    void DummyVisualizationSet::setUpdateVisualization(bool enable)
    {
        DummyVisualization::setUpdateVisualization(enable);
        VisualizationSet::setUpdateVisualization(enable);
    }

    bool DummyVisualizationSet::getUpdateVisualizationStatus() const
    {
        return DummyVisualization::getUpdateVisualizationStatus();
    }

    void DummyVisualizationSet::setStyle(Visualization::DrawStyle s)
    {
        DummyVisualization::setStyle(s);
        VisualizationSet::setStyle(s);
    }

    Visualization::DrawStyle DummyVisualizationSet::getStyle() const
    {
        return DummyVisualization::getStyle();
    }

    void DummyVisualizationSet::setColor(const Visualization::Color &c)
    {
        DummyVisualization::setColor(c);
        VisualizationSet::setColor(c);
    }

    Visualization::Color DummyVisualizationSet::getColor() const
    {
        return DummyVisualization::getColor();
    }

    void DummyVisualizationSet::setMaterial(const Visualization::MaterialPtr &material)
    {
        DummyVisualization::setMaterial(material);
        VisualizationSet::setMaterial(material);
    }

    Visualization::MaterialPtr DummyVisualizationSet::getMaterial() const
    {
        return DummyVisualization::getMaterial();
    }

    void DummyVisualizationSet::_setSelected(bool selected)
    {
        DummyVisualization::_setSelected(selected);
        VisualizationSet::_setSelected(selected);
    }

    bool DummyVisualizationSet::isSelected() const
    {
        return DummyVisualization::isSelected();
    }

    void DummyVisualizationSet::setScalingFactor(const Eigen::Vector3f &scaleFactor)
    {
        DummyVisualization::setScalingFactor(scaleFactor);
        VisualizationSet::setScalingFactor(scaleFactor);
    }

    Eigen::Vector3f DummyVisualizationSet::getScalingFactor() const
    {
        return DummyVisualization::getScalingFactor();
    }

    void DummyVisualizationSet::shrinkFatten(float offset)
    {
        DummyVisualization::shrinkFatten(offset);
        VisualizationSet::shrinkFatten(offset);
    }

    void DummyVisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
        DummyVisualization::_addManipulator(t);
        VisualizationSet::_addManipulator(t);
    }

    void DummyVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        DummyVisualization::_removeManipulator(t);
        VisualizationSet::_removeManipulator(t);
    }

    bool DummyVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
        return DummyVisualization::hasManipulator(t);
    }

    std::vector<Visualization::ManipulatorType> DummyVisualizationSet::getAddedManipulatorTypes() const
    {
        return DummyVisualization::getAddedManipulatorTypes();
    }

    void DummyVisualizationSet::_removeAllManipulators()
    {
        DummyVisualization::_removeAllManipulators();
        VisualizationSet::_removeAllManipulators();
    }

    std::vector<Primitive::PrimitivePtr> DummyVisualizationSet::getPrimitives() const
    {
        return DummyVisualization::getPrimitives();
    }

    void DummyVisualizationSet::setFilename(const std::string &filename, bool boundingBox)
    {
        DummyVisualization::setFilename(filename, boundingBox);
    }

    std::string DummyVisualizationSet::getFilename() const
    {
        return DummyVisualization::getFilename();
    }

    bool DummyVisualizationSet::usedBoundingBoxVisu() const
    {
        return DummyVisualization::usedBoundingBoxVisu();
    }

    BoundingBox DummyVisualizationSet::getBoundingBox() const
    {
        return VisualizationSet::getBoundingBox();
    }

    TriMeshModelPtr DummyVisualizationSet::getTriMeshModel() const
    {
        return VisualizationSet::getTriMeshModel();
    }

    void DummyVisualizationSet::createTriMeshModel()
    {
        VisualizationSet::createTriMeshModel();
    }

    int DummyVisualizationSet::getNumFaces() const
    {
        return VisualizationSet::getNumFaces();
    }

    void DummyVisualizationSet::print() const
    {
        VisualizationSet::print();
    }

    std::string DummyVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        return VisualizationSet::toXML(basePath, tabs);
    }

    std::string DummyVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        return VisualizationSet::toXML(basePath, filename, tabs);
    }

    bool DummyVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        return VisualizationSet::saveModel(modelPath, filename);
    }

} // namespace VirtualRobot
