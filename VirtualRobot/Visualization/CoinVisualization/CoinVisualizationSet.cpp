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
          usedBoundingBox(false)
    {
        setNode->ref();
        for (auto& visu : this->getVisualizations())
        {
            auto visuCoin = visualization_cast<CoinElement>(visu);
            setNode->addChild(visuCoin->getMainNode());
        }
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
    }

    bool CoinVisualizationSet::removeVisualization(const VisualizationPtr &visu)
    {
        if (VisualizationSet::removeVisualization(visu))
        {
            auto visuCoin = visualization_cast<CoinElement>(visu);
            setNode->removeChild(visuCoin->getMainNode());
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
        return VisualizationFactory::getInstance()->createVisualisationSet(clonedVisus);
    }

    void CoinVisualizationSet::setSelected(bool selected)
    {
        VisualizationSet::setSelected(selected);
    }

    bool CoinVisualizationSet::isSelected() const
    {
        return VisualizationSet::isSelected();
    }

    void CoinVisualizationSet::_addManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR_ONCE_NYI;
    }

    void CoinVisualizationSet::_removeManipulator(Visualization::ManipulatorType t)
    {
        VR_ERROR_ONCE_NYI;
    }

    void CoinVisualizationSet::_removeAllManipulators()
    {
        VR_ERROR_ONCE_NYI;
    }

    bool CoinVisualizationSet::hasManipulator(Visualization::ManipulatorType t) const
    {
        VR_ERROR_ONCE_NYI;
        return false;
    }

    std::vector<Visualization::ManipulatorType> CoinVisualizationSet::getAddedManipulatorTypes() const
    {
        VR_ERROR_ONCE_NYI;
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


    std::string CoinVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        VR_ERROR_ONCE_NYI;
        return "";
    }

    std::string CoinVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        VR_ERROR_ONCE_NYI;
        return "";
    }

    bool CoinVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        VR_ERROR_ONCE_NYI;
        return false;
    }
} // namespace VirtualRobot
