/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Manfred Kroehnert
*/


#include "VisualizationNode.h"
#include "TriMeshModel.h"

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/XML/BaseIO.h"

namespace
{
    namespace fs = std::filesystem;
    inline fs::path remove_trailing_separator(fs::path p)
    {
        p /= "dummy";
        return p.parent_path();
    }
}

namespace VirtualRobot
{

    VisualizationNode::VisualizationNode(const TriMeshModelPtr& triMeshModel) :
        triMeshModel{triMeshModel}
    {
        updateVisualization = true;
        showVisualization = true;
        showAttachedVisualizations = true;
        boundingBox = false;
        globalPose.setIdentity();
    }

    VisualizationNode::~VisualizationNode()
    {
        attachedVisualizations.clear();
    }

    VirtualRobot::VisualizationNodePtr VisualizationNode::clone(bool deepCopy, float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling <= 0, "Scaling must be >0");

        VisualizationNodePtr p(new VisualizationNode());

        if (deepCopy && triMeshModel)
        {
            p->triMeshModel = triMeshModel->clone();
            p->triMeshModel->scale(scaling);
        }
        else
        {
            p->triMeshModel = triMeshModel;
        }


        p->setUpdateVisualization(updateVisualization);
        p->setGlobalPose(getGlobalPose());
        p->setFilename(filename, boundingBox);

        for (const auto& [key, value] : attachedVisualizations)
        {
            VisualizationNodePtr attachedClone = value->clone(deepCopy, scaling);
            p->attachVisualization(key, attachedClone);
        }

        p->primitives = primitives;

        return p;
    }

    VirtualRobot::TriMeshModelPtr VisualizationNode::getTriMeshModel()
    {
        return triMeshModel;
    }

    void VisualizationNode::attachVisualization(const std::string& name, VisualizationNodePtr v)
    {
        THROW_VR_EXCEPTION_IF(!v, "NULL DATA");

        attachedVisualizations[name] = v;
    }

    void VisualizationNode::detachVisualization(const std::string& name)
    {
        attachedVisualizations.erase(name);
    }

    bool VisualizationNode::hasAttachedVisualization(const std::string& name)
    {
        std::map< std::string, VisualizationNodePtr >::const_iterator i = attachedVisualizations.begin();

        while (i != attachedVisualizations.end())
        {
            if (i->first == name)
            {
                return true;
            }

            i++;
        }

        return false;
    }

    VisualizationNodePtr VisualizationNode::getAttachedVisualization(const std::string& name)
    {
        if (!hasAttachedVisualization(name))
        {
            return VisualizationNodePtr();
        }

        return attachedVisualizations[name];
    }


    void VisualizationNode::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;
    }

    bool VisualizationNode::getUpdateVisualizationStatus()
    {
        return updateVisualization;
    }

    void VisualizationNode::print()
    {
        cout << "Dummy VisualizationNode" << endl;
    }

    void VisualizationNode::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        this->showVisualization = showVisualization;
        this->showAttachedVisualizations = showAttachedVisualizations;
    }

    int VisualizationNode::getNumFaces()
    {
        TriMeshModelPtr p = getTriMeshModel();

        if (p)
        {
            return (int)p->faces.size();
        }

        return 0;
    }

    void VisualizationNode::setFilename(const std::string& filename, bool boundingBox)
    {
        this->filename = filename;
        this->boundingBox = boundingBox;
    }

    bool VisualizationNode::usedBoundingBoxVisu()
    {
        return boundingBox;
    }

    std::string VisualizationNode::getFilename()
    {
        return filename;
    }

    std::string VisualizationNode::toXML(const std::string& basePath, const std::string& filename, int tabs)
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
            ss << pre << t << "<File type='" << getType() << "'>" << tmpFilename << "</File>\n";
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

    std::string VisualizationNode::toXML(const std::string& basePath, int tabs)
    {
        std::string visualizationFilename = getFilename();
        std::filesystem::path fn(visualizationFilename);
        return toXML(basePath, fn.string(), tabs);
    }

    VirtualRobot::VisualizationNodePtr VisualizationNode::CreateUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations)
    {
        if (visualizations.size() == 0)
        {
            return VisualizationNodePtr();
        }

        VisualizationFactoryPtr f;
        std::vector<VisualizationNodePtr>::const_iterator i = visualizations.begin();

        while (!f && i != visualizations.end())
        {
            if ((*i)->getType() != VisualizationFactory::getName())
            {
                f = VisualizationFactory::fromName((*i)->getType(), NULL);
                break;
            }

            i++;
        }

        if (i == visualizations.end())
        {
            VR_ERROR << "Could not find visualization factory. Aborting..." << endl;
            return VisualizationNodePtr();
        }

        THROW_VR_EXCEPTION_IF(!f, "No VisualizationFactory");

        return f->createUnitedVisualization(visualizations);
    }

    VirtualRobot::BoundingBox VisualizationNode::getBoundingBox()
    {
        VirtualRobot::BoundingBox bbox;
        TriMeshModelPtr tm = getTriMeshModel();

        if (!tm)
        {
            return bbox;
        }

        bbox = tm->boundingBox;
        bbox.transform(globalPose);
        return bbox;
    }

    bool VisualizationNode::saveModel(const std::string& modelPath, std::string& filename)
    {
        const auto completePath = remove_trailing_separator(modelPath);

        if (!std::filesystem::is_directory(completePath))
        {
            if (!std::filesystem::create_directories(completePath))
            {
                VR_ERROR << "Could not create model dir  " << completePath.string() << endl;
                return false;
            }
        }
        const auto completeFile = std::filesystem::absolute(completePath / filename).replace_extension("off");

        const auto& t = *getTriMeshModel();
        VR_INFO << "writing " << completeFile.string() << std::endl;
        std::ofstream out{completeFile.string()};
        out << "OFF\n# num vert / num face / num edge\n"
            << t.vertices.size() << ' ' << t.faces.size()
            << " 0\n\n#vert (x y z)\n";
        for (const auto& v : t.vertices)
        {
            out << v.x() <<  ' ' << v.y() <<  ' ' << v.z() <<  '\n';
        }
        out << "\n# face (num vert N v0idx v1idx ... vNidx)\n";
        for (const auto& f : t.faces)
        {
            out << "3 " << f.id1 <<  ' ' << f.id2 <<  ' ' << f.id3 <<  '\n';
        }

        return true;
    }

    void VisualizationNode::scale(const Eigen::Vector3f& scaleFactor)
    {
        if (triMeshModel)
        {
            triMeshModel->scale(scaleFactor);
        }
    }
    void VisualizationNode::scale(float scaleFactor)
    {
        if (triMeshModel)
        {
            triMeshModel->scale(Eigen::Vector3f{scaleFactor, scaleFactor, scaleFactor});
        }
    }

    void VisualizationNode::shrinkFatten(float offset)
    {
        if (offset != 0.0f && triMeshModel)
        {
            triMeshModel->fattenShrink(offset, true);
        }
    }

    void VisualizationNode::createTriMeshModel()
    {

    }

} // namespace VirtualRobot
