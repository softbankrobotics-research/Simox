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
        : Visualization()
    {
        for (auto& visu : visualizations)
        {
            addVisualization(visu);
        }
    }

    VisualizationSet::~VisualizationSet()
    {
    }

    void VisualizationSet::addVisualization(const VisualizationPtr &visu)
    {
        if (!containsVisualization(visu))
        {
            visualizations.push_back(visu);
        }
    }

    bool VisualizationSet::containsVisualization(const VisualizationPtr &visu) const
    {
        return std::find(visualizations.begin(), visualizations.end(), visu) != visualizations.end();
    }

    bool VisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {
        auto it = std::find(visualizations.begin(), visualizations.end(), visu);
        if (it != visualizations.end())
        {
            visualizations.erase(it);
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
        return visualizations;
    }

    VisualizationPtr VisualizationSet::at(size_t index) const
    {
        return visualizations.at(index);
    }

    VisualizationPtr VisualizationSet::operator[](size_t index) const
    {
        return visualizations[index];
    }

    bool VisualizationSet::empty() const
    {
        return visualizations.empty();
    }

    size_t VisualizationSet::size() const
    {
        return visualizations.size();
    }

    void VisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Eigen::Matrix4f oldM = this->getGlobalPose();
        Eigen::Matrix4f dp = m * oldM.inverse();
        for (auto& visu : visualizations)
        {
            visu->setGlobalPose(dp * visu->getGlobalPose());
        }
        setGlobalPoseNoUpdate(m);
    }

    void VisualizationSet::setGlobalPoseNoUpdate(const Eigen::Matrix4f &m)
    {
        Visualization::setGlobalPose(m);
    }

    void VisualizationSet::applyDisplacement(const Eigen::Matrix4f &dp)
    {
        setGlobalPose(getGlobalPose()*dp);
    }

    void VisualizationSet::setVisible(bool showVisualization)
    {
        for (auto& visu : visualizations)
        {
            visu->setVisible(showVisualization);
        }
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

    void VisualizationSet::setUpdateVisualization(bool enable)
    {
        for (auto& visu : visualizations)
        {
            visu->setUpdateVisualization(enable);
        }
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

    void VisualizationSet::setStyle(Visualization::DrawStyle s)
    {
        for (auto& visu : visualizations)
        {
            visu->setStyle(s);
        }
    }

    Visualization::DrawStyle VisualizationSet::getStyle() const
    {
        if (visualizations.empty())
        {
            return Visualization::DrawStyle::undefined;
        }
        Visualization::DrawStyle s = visualizations[0]->getStyle();
        for (auto& visu : visualizations)
        {
            if (s != visu->getStyle())
            {
                return Visualization::DrawStyle::undefined;
            }
        }
        return s;
    }

    void VisualizationSet::setColor(const Visualization::Color &c)
    {
        for (auto& visu : visualizations)
        {
            visu->setColor(c);
        }
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

    void VisualizationSet::setMaterial(const MaterialPtr &material)
    {
        for (auto& visu : visualizations)
        {
            visu->setMaterial(material);
        }
    }

    Visualization::MaterialPtr VisualizationSet::getMaterial() const
    {
        if (visualizations.empty())
        {
            return Visualization::MaterialPtr(new Visualization::NoneMaterial);
        }
        Visualization::MaterialPtr m = visualizations[0]->getMaterial();
        for (auto& visu : visualizations)
        {
            if (m != visu->getMaterial())
            {
                return Visualization::MaterialPtr(new Visualization::NoneMaterial);
            }
        }
        return m;
    }

    void VisualizationSet::setSelected(bool selected)
    {
        Visualization::setSelected(selected);
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

    void VisualizationSet::scale(const Eigen::Vector3f &scaleFactor)
    {
        Eigen::Vector3f gpos = getGlobalPosition();
        for (auto& visu : visualizations)
        {
            Eigen::Matrix4f visuGp = visu->getGlobalPose();
            visuGp.block<3, 1>(0, 3) = gpos + (visuGp.block<3, 1>(0, 3) - gpos).cwiseProduct(scaleFactor);
            visu->setGlobalPose(visuGp);
            visu->scale(scaleFactor);
        }
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
        BoundingBox b;
        for (auto& visu : visualizations)
        {
            b.addPoints(visu->getBoundingBox().getPoints());
        }
        return b;
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
        int n = 0;
        for (auto& visu : visualizations)
        {
            n += visu->getNumFaces();
        }
        return n;
    }

    void VisualizationSet::print() const
    {
        VR_ERROR_ONCE_NYI;
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
        VR_ERROR_ONCE_NYI;
        return "";
    }

    std::string DummyVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        VR_ERROR_ONCE_NYI;
        return "";
    }

    bool DummyVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        VR_ERROR_ONCE_NYI;
        return false;
    }

} // namespace VirtualRobot
