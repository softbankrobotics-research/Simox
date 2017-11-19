/**
* @package    VirtualRobot
* @author     Adrian Knobloch
* @copyright  2017 Adrian Knobloch
*/

#include "VisualizationGroup.h"
#include "Visualization.h"
#include "VisualizationFactory.h"

namespace VirtualRobot
{
    VisualizationGroup::VisualizationGroup()
    {
    }

    VisualizationGroup::VisualizationGroup(const std::vector<VisualizationPtr> &visualizations) : Frame(), visualizations(visualizations)
    {
    }

    VisualizationGroup::~VisualizationGroup()
    {
    }

    void VisualizationGroup::addVisualization(const VisualizationPtr &visu)
    {
        if (!containsVisualization(visu))
        {
            visualizations.push_back(visu);
        }
    }

    bool VisualizationGroup::containsVisualization(const VisualizationPtr &visu) const
    {
        return std::find(visualizations.begin(), visualizations.end(), visu) != visualizations.end();
    }

    bool VisualizationGroup::removeVisualization(const VisualizationPtr &visu)
    {
        auto it = std::find(visualizations.begin(), visualizations.end(), visu);
        if (it != visualizations.end())
        {
            visualizations.erase(it);
            return true;
        }
        return false;
    }

    bool VisualizationGroup::removeVisualization(size_t id)
    {
        return this->removeVisualization(this->at(id));
    }

    std::vector<VisualizationPtr> VisualizationGroup::getVisualizations() const
    {
        return visualizations;
    }

    VisualizationPtr VisualizationGroup::at(size_t index) const
    {
        return visualizations.at(index);
    }

    VisualizationPtr VisualizationGroup::operator[](size_t index) const
    {
        return visualizations[index];
    }

    bool VisualizationGroup::empty() const
    {
        return visualizations.empty();
    }

    size_t VisualizationGroup::size() const
    {
        return visualizations.size();
    }

    void VisualizationGroup::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Eigen::Matrix4f oldM = this->getGlobalPose();
        Eigen::Matrix4f dp = m * oldM.inverse();
        for (auto& visu : visualizations)
        {
            visu->setGlobalPose(dp * visu->getGlobalPose());
        }
        setGlobalPoseNoUpdate(m);
    }

    void VisualizationGroup::setGlobalPoseNoUpdate(const Eigen::Matrix4f &m)
    {
        globalPose = m;
    }

    void VisualizationGroup::applyDisplacement(const Eigen::Matrix4f &dp)
    {
        setGlobalPose(getGlobalPose()*dp);
    }

    void VisualizationGroup::setVisible(bool showVisualization)
    {
        for (auto& visu : visualizations)
        {
            visu->setVisible(showVisualization);
        }
    }

    bool VisualizationGroup::isVisible() const
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

    void VisualizationGroup::setUpdateVisualization(bool enable)
    {
        for (auto& visu : visualizations)
        {
            visu->setUpdateVisualization(enable);
        }
    }

    bool VisualizationGroup::getUpdateVisualizationStatus() const
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

    void VisualizationGroup::setStyle(Visualization::DrawStyle s)
    {
        for (auto& visu : visualizations)
        {
            visu->setStyle(s);
        }
    }

    Visualization::DrawStyle VisualizationGroup::getStyle() const
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

    void VisualizationGroup::setColor(const Visualization::Color &c)
    {
        for (auto& visu : visualizations)
        {
            visu->setColor(c);
        }
    }

    Visualization::Color VisualizationGroup::getColor() const
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

    void VisualizationGroup::setMaterial(const Visualization::MaterialPtr &material)
    {
        for (auto& visu : visualizations)
        {
            visu->setMaterial(material);
        }
    }

    Visualization::MaterialPtr VisualizationGroup::getMaterial() const
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

    void VisualizationGroup::scale(const Eigen::Vector3f &scaleFactor)
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

    BoundingBox VisualizationGroup::getBoundingBox() const
    {
        BoundingBox b;
        for (auto& visu : visualizations)
        {
            b.addPoints(visu->getBoundingBox().getPoints());
        }
        return b;
    }

    TriMeshModelPtr VisualizationGroup::getTriMeshModel() const
    {
        // TODO
    }

    std::vector<Primitive::PrimitivePtr> VisualizationGroup::getPrimitives() const
    {
        // TODO
    }

    int VisualizationGroup::getNumFaces() const
    {
        int n = 0;
        for (auto& visu : visualizations)
        {
            n += visu->getNumFaces();
        }
        return n;
    }

    void VisualizationGroup::print() const
    {
        // TODO
    }
} // namespace VirtualRobot
