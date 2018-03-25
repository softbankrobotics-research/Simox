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

    bool VisualizationSet::containsVisualization(const VisualizationPtr &visu) const
    {
        return VisualizationGroup::containsVisualization(visu);
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

    bool VisualizationSet::removeVisualization(size_t index)
    {
        return this->removeVisualization(this->at(index));
    }

    std::vector<VisualizationPtr> VisualizationSet::getVisualizations() const
    {
        return VisualizationGroup::getVisualizations();
    }

    VisualizationPtr VisualizationSet::at(size_t index) const
    {
        return VisualizationGroup::at(index);
    }

    VisualizationPtr VisualizationSet::operator[](size_t index) const
    {
        return VisualizationGroup::operator [](index);
    }

    bool VisualizationSet::empty() const
    {
        return VisualizationGroup::empty();
    }

    size_t VisualizationSet::size() const
    {
        return VisualizationGroup::size();
    }

    Eigen::Matrix4f VisualizationSet::getGlobalPose() const
    {
        return VisualizationGroup::getGlobalPose();
    }

    void VisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationGroup::setGlobalPose(m);
    }

    void VisualizationSet::setGlobalPoseNoUpdate(const Eigen::Matrix4f &m)
    {
        VisualizationGroup::setGlobalPoseNoUpdate(m);
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

    Eigen::Vector3f transformPosition(const Eigen::Matrix4f& transform, const Eigen::Vector3f& pos)
    {
        return (transform * Eigen::Vector4f(pos.x(), pos.y(), pos.z(), 1)).block<3, 1>(0, 0);
    }

    TriMeshModelPtr VisualizationSet::getTriMeshModel() const
    {
        TriMeshModelPtr mesh(new TriMeshModel);
        for (const auto& visu : getVisualizations())
        {
            const auto& tm = visu->getTriMeshModel();
            Eigen::Matrix4f transform = visu->getGlobalPose() * getGlobalPose().inverse();
            for (const auto& face : tm->faces)
            {
                TriangleFace cloned;
                cloned.id1 = mesh->addVertex(transformPosition(transform, tm->vertices.at(face.id1)));
                cloned.id2 = mesh->addVertex(transformPosition(transform, tm->vertices.at(face.id2)));
                cloned.id3 = mesh->addVertex(transformPosition(transform, tm->vertices.at(face.id3)));

                cloned.normal = face.normal;

                if (tm->colors.size() > face.idColor1 && tm->colors.size() > face.idColor2 && tm->colors.size() > face.idColor3)
                {
                    cloned.idColor1 = mesh->addColor(tm->colors.at(face.idColor1));
                    cloned.idColor2 = mesh->addColor(tm->colors.at(face.idColor2));
                    cloned.idColor3 = mesh->addColor(tm->colors.at(face.idColor3));
                }

                if (tm->normals.size() > face.idNormal1 && tm->normals.size() > face.idNormal2 && tm->normals.size() > face.idNormal3)
                {
                    cloned.idNormal1 = mesh->addNormal(tm->normals.at(face.idNormal1));
                    cloned.idNormal2 = mesh->addNormal(tm->normals.at(face.idNormal2));
                    cloned.idNormal3 = mesh->addNormal(tm->normals.at(face.idNormal3));
                }

                if (tm->materials.size() > face.idMaterial)
                {
                    cloned.idMaterial = mesh->addMaterial(tm->materials.at(face.idMaterial));
                }

                mesh->addFace(cloned);
            }
        }
        return mesh;
    }

    int VisualizationSet::getNumFaces() const
    {
        return VisualizationGroup::getNumFaces();
    }

    void VisualizationSet::print() const
    {
        VisualizationGroup::print();
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
        return VisualizationFactory::getInstance()->createVisualisationSet(clonedVisus);
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
        static bool printed = false;
        if (!printed)
        {
            VR_ERROR << __FILE__ << " " << __LINE__ << ": NYI" << std::endl;
            printed = true;
        }
        return 0;
    }

    void DummyVisualizationSet::removeSelectionChangedCallback(size_t id)
    {
        static bool printed = false;
        if (!printed)
        {
            VR_ERROR << __FILE__ << " " << __LINE__ << ": NYI" << std::endl;
            printed = true;
        }
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
