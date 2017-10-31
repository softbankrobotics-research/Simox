/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Manfred Kroehnert
*/


#include "Visualization.h"
#include "TriMeshModel.h"
#include "../Tools/BoundingBox.h"

#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/XML/BaseIO.h"


namespace VirtualRobot
{

    DummyVisualization::DummyVisualization() :
        Visualization(),
        visible(true),
        updateVisualization(true),
        style(Visualization::DrawStyle::normal),
        material(),
        selected(false),
        addedManipulators(),
        filename(""),
        boundingBox(false),
        primitives(),
        inVisualizationSet(false),
        scaleFactor(Eigen::Vector3f(1.f, 1.f, 1.f))
    {
        setColor(Visualization::Color::None());
    }

    void Visualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        globalPose = m;
    }

    void Visualization::applyDisplacement(const Eigen::Matrix4f &dp)
    {
        this->setGlobalPose(this->getGlobalPose() * dp);
    }

    bool Visualization::isInVisualizationSet() const
    {
        return inVisualizationSet;
    }

    void Visualization::setIsInVisualizationSet(bool inSet)
    {
        inVisualizationSet = inSet;
    }

    void DummyVisualization::setGlobalPose(const Eigen::Matrix4f &m)
    {
        Visualization::setGlobalPose(m);
        for (auto& f : poseChangedCallbacks)
        {
            f.second(m);
        }
    }

    size_t DummyVisualization::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        static unsigned int id = 0;
        poseChangedCallbacks[id] = f;
        return id++;
    }

    void DummyVisualization::removePoseChangedCallback(size_t id)
    {
        auto it = poseChangedCallbacks.find(id);
        if (it != poseChangedCallbacks.end())
        {
            poseChangedCallbacks.erase(it);
        }
    }

    void DummyVisualization::setVisible(bool showVisualization)
    {
        visible = showVisualization;
    }

    bool DummyVisualization::isVisible() const
    {
        return visible;
    }

    void DummyVisualization::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;
    }

    bool DummyVisualization::getUpdateVisualizationStatus() const
    {
        return updateVisualization;
    }

    void DummyVisualization::setStyle(Visualization::DrawStyle s)
    {
        style = s;
    }

    Visualization::DrawStyle DummyVisualization::getStyle() const
    {
        return style;
    }

    void DummyVisualization::setColor(const Visualization::Color &c)
    {
        material.diffuse = c;
        material.ambient = c;
        setMaterial(material);
    }

    Visualization::Color DummyVisualization::getColor() const
    {
        return material.diffuse;
    }

    void DummyVisualization::setMaterial(const Visualization::PhongMaterial &material)
    {
        this->material = material;
    }

    Visualization::PhongMaterial DummyVisualization::getMaterial() const
    {
        return material;
    }

    void DummyVisualization::_setSelected(bool selected)
    {
        this->selected = selected;
        for (auto& f : selectionChangedCallbacks)
        {
            f.second(selected);
        }
    }

    bool DummyVisualization::isSelected() const
    {
        return selected;
    }

    size_t DummyVisualization::addSelectionChangedCallback(std::function<void (bool)> f)
    {
        static unsigned int id = 0;
        selectionChangedCallbacks[id] = f;
        return id++;
    }

    void DummyVisualization::removeSelectionChangedCallback(size_t id)
    {
        auto it = selectionChangedCallbacks.find(id);
        if (it != selectionChangedCallbacks.end())
        {
            selectionChangedCallbacks.erase(it);
        }
    }

    void DummyVisualization::scale(const Eigen::Vector3f &scaleFactor)
    {
        this->scaleFactor = scaleFactor;
    }

    Eigen::Vector3f DummyVisualization::getScaleFactor() const
    {
        return scaleFactor;
    }

    void DummyVisualization::shrinkFatten(float offset)
    {
        this->createTriMeshModel();
        this->getTriMeshModel()->mergeVertices();
        this->getTriMeshModel()->fattenShrink(offset);
    }

    void DummyVisualization::_addManipulator(Visualization::ManipulatorType t)
    {
        addedManipulators.insert(t);
    }

    void DummyVisualization::_removeManipulator(Visualization::ManipulatorType t)
    {
        auto pos = addedManipulators.find(t);
        if (pos != addedManipulators.end())
        {
            addedManipulators.erase(pos);
        }
    }

    void DummyVisualization::_removeAllManipulators()
    {
        addedManipulators.clear();
    }

    bool DummyVisualization::hasManipulator(Visualization::ManipulatorType t) const
    {
        auto pos = addedManipulators.find(t);
        return pos != addedManipulators.end();
    }

    std::vector<Visualization::ManipulatorType> DummyVisualization::getAddedManipulatorTypes() const
    {
        return std::vector<ManipulatorType>(addedManipulators.begin(), addedManipulators.end());
    }

    std::vector<Primitive::PrimitivePtr> DummyVisualization::getPrimitives() const
    {
        return primitives;
    }

    void DummyVisualization::setFilename(const std::string &filename, bool boundingBox)
    {
        this->filename = filename;
        this->boundingBox = boundingBox;
    }

    std::string DummyVisualization::getFilename() const
    {
        return filename;
    }

    bool DummyVisualization::usedBoundingBoxVisu() const
    {
        return boundingBox;
    }

    BoundingBox DummyVisualization::getBoundingBox() const
    {
        TriMeshModelPtr tm = this->getTriMeshModel();
        VR_ASSERT(tm);

        VirtualRobot::BoundingBox bbox = tm->boundingBox;
        bbox.transform(this->getGlobalPose());
        return bbox;
    }

    TriMeshModelPtr DummyVisualization::getTriMeshModel() const
    {
        VR_ASSERT(triMeshModel);
        return triMeshModel;
    }

    void DummyVisualization::createTriMeshModel()
    {
        triMeshModel.reset(new TriMeshModel);
    }

    int DummyVisualization::getNumFaces() const
    {
        TriMeshModelPtr p = getTriMeshModel();
        VR_ASSERT(p);

        return (int)p->faces.size();
    }

    VisualizationPtr DummyVisualization::clone(float scaling) const
    {
        VisualizationPtr visu(new DummyVisualization);

        visu->setVisible(this->isVisible());
        visu->setUpdateVisualization(this->getUpdateVisualizationStatus());
        visu->setStyle(this->getStyle());
        visu->setColor(this->getColor());
        visu->setFilename(this->getFilename(), this->usedBoundingBoxVisu());
        visu->scale(this->getScaleFactor() * scaling);
        visu->createTriMeshModel();

        return VisualizationPtr();
    }

    void DummyVisualization::print() const
    {
        std::cout << "Dummy VisualizationNode" << std::endl;
    }

    std::string DummyVisualization::toXML(const std::string &basePath, int tabs) const
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<Visualization";

        if (usedBoundingBoxVisu())
        {
            ss << " BoundingBox='true'";
        }

        ss << ">\n";

        if (!filename.empty())
        {
            std::string tmpFilename = filename;
            BaseIO::makeRelativePath(basePath, tmpFilename);
            ss << pre << t << "<File type='" << VisualizationFactory::getGlobalVisualizationFactory()->getVisualizationType() << "'>" << tmpFilename << "</File>\n";
        }
        else if (primitives.size() != 0)
        {
            ss << pre << "\t<Primitives>\n";
            std::vector<Primitive::PrimitivePtr>::const_iterator it;

            for (it = primitives.begin(); it != primitives.end(); it++)
            {
                ss << (*it)->toXMLString(tabs + 1);
            }

            ss << pre << "\t</Primitives>\n";
        }

        ss << pre << "</Visualization>\n";

        return ss.str();
    }

    std::string DummyVisualization::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        std::string visualizationFilename = getFilename();
        boost::filesystem::path fn(visualizationFilename);
        return toXML(basePath, fn.string(), tabs);
    }

    bool DummyVisualization::saveModel(const std::string &modelPath, const std::string &filename)
    {
        // derived classes have to overwrite this method, otherwise a NYI will show up
        VR_ERROR << "NYI..." << endl;
        return false;
    }

} // namespace VirtualRobot
