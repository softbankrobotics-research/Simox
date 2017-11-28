/**
* @package    VirtualRobot
* @author     Manfred Kroehnert, Adrian Knobloch
* @copyright  2010, 2017 Manfred Kroehnert, Adrian Knobloch
*/


#include "CoinVisualizationSet.h"
#include "CoinVisualizationFactory.h"
#include "CoinVisualization.h"
#include "../../VirtualRobotException.h"
#include "../TriMeshModel.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoShape.h>

#include <algorithm>

// For the VRML2.0 export
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/nodes/SoRotation.h>

namespace VirtualRobot
{   
    CoinVisualizationSet::CoinVisualizationSet(const std::vector<VisualizationPtr> &visualizations)
        : VisualizationSet(visualizations),
          setNode(new SoSeparator),
          filename(""),
          usedBoundingBox(false),
          triMeshModel(nullptr)
    {
        setNode->ref();
        for (auto& visu : this->getVisualizations())
        {
            auto visuCoin = visualization_cast<CoinElement>(visu);
            setNode->addChild(visuCoin->getMainNode());
        }
        createTriMeshModel();
    }

    CoinVisualizationSet::~CoinVisualizationSet()
    {
        setNode->removeAllChildren();
        setNode->unref();
    }

    SoNode *CoinVisualizationSet::getMainNode() const
    {
        return setNode;
    }

    void CoinVisualizationSet::addVisualization(const VisualizationPtr &visu)
    {
        if (!containsVisualization(visu))
        {
            auto visuCoin = visualization_cast<CoinElement>(visu);
            setNode->addChild(visuCoin->getMainNode());
            VisualizationSet::addVisualization(visu);
        }
        createTriMeshModel();
    }

    bool CoinVisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {
        if (VisualizationSet::removeVisualization(visu))
        {
            auto visuCoin = visualization_cast<CoinElement>(visu);
            setNode->removeChild(visuCoin->getMainNode());
            createTriMeshModel();
            return true;
        }
        return false;
    }

    VisualizationPtr CoinVisualizationSet::clone() const
    {
        std::vector<VisualizationPtr> clonedVisus;
        const auto& visus = getVisualizations();
        clonedVisus.reserve(visus.size());
        for (const auto& visu : visus)
        {
            clonedVisus.push_back(visu->clone());
        }
        return VisualizationFactory::getGlobalVisualizationFactory()->createVisualisationSet(clonedVisus);
    }

    void CoinVisualizationSet::setGlobalPose(const Eigen::Matrix4f &m)
    {
        VisualizationSet::setGlobalPose(m);
        for (auto& f : poseChangedCallbacks)
        {
            f.second(m);
        }
    }

    size_t CoinVisualizationSet::addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f)
    {
        static unsigned int id = 0;
        poseChangedCallbacks[id] = f;
        return id++;
    }

    void CoinVisualizationSet::removePoseChangedCallback(size_t id)
    {
        auto it = poseChangedCallbacks.find(id);
        if (it != poseChangedCallbacks.end())
        {
            poseChangedCallbacks.erase(it);
        }
    }

    void CoinVisualizationSet::setSelected(bool selected)
    {
        VisualizationSet::setSelected(selected);
    }

    bool CoinVisualizationSet::isSelected() const
    {
        return VisualizationSet::isSelected();
    }

    size_t CoinVisualizationSet::addSelectionChangedCallback(std::function<void (bool)> f)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualizationSet::removeSelectionChangedCallback(size_t id)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR << "NYI" << std::endl;
    }

    void CoinVisualizationSet::_removeAllManipulators()
    {
        VR_ERROR << "NYI" << std::endl;
    }

    bool CoinVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
        VR_ERROR << "NYI" << std::endl;
        return false;
    }

    std::vector<Visualization::ManipulatorType> CoinVisualizationSet::getAddedManipulatorTypes() const
    {
        VR_ERROR << "NYI" << std::endl;
        return std::vector<ManipulatorType>();
    }

    void CoinVisualizationSet::setFilename(const std::string &filename, bool boundingBox)
    {
        this->filename = filename;
        this->usedBoundingBox = boundingBox;
    }

    std::string CoinVisualizationSet::getFilename() const
    {
        return filename;
    }

    bool CoinVisualizationSet::usedBoundingBoxVisu() const
    {
        return usedBoundingBox;
    }

    TriMeshModelPtr CoinVisualizationSet::getTriMeshModel() const
    {
        VR_ASSERT(triMeshModel);
        return triMeshModel;
    }

    void CoinVisualizationSet::createTriMeshModel()
    {
        THROW_VR_EXCEPTION_IF(!getMainNode(), "CoinVisualizationNode::createTriMeshModel(): no Coin model present!");

        if (triMeshModel)
        {
            triMeshModel->clear();
        }
        else
        {
            triMeshModel.reset(new TriMeshModel());
        }

        SoCallbackAction ca;
        ca.addTriangleCallback(SoShape::getClassTypeId(), &CoinVisualization::InventorTriangleCB, triMeshModel.get());
        ca.apply(getMainNode());
    }

    std::string CoinVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        VR_ERROR << "NYI" << std::endl;
        return "";
    }

    std::string CoinVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        VR_ERROR << "NYI" << std::endl;
        return "";
    }

    bool CoinVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        VR_ERROR << "NYI" << std::endl;
        return false;
    }
} // namespace VirtualRobot
