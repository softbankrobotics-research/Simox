/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    Gui
* @author     Peter Kaiser, Adrian Knobloch
* @copyright  2016,2017 Peter Kaiser, Adrian Knobloch
*             GNU Lesser General Public License
*
*/

#include "CoinViewer.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationSet.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinSelectionGroup.h>
#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/Qt/SoQt.h>
#include <QGLWidget>

#include <iostream>

namespace SimoxGui
{
    CoinViewer::CoinViewer(QWidget *parent)
        : SoQtExaminerViewer(parent, "", true, BUILD_POPUP),
          parent(parent),
          sceneSep(new SoSeparator),
          unitNode(new SoUnits)
    {
        sceneSep->ref();
        unitNode->ref();

        unitNode->units = SoUnits::MILLIMETERS;
        sceneSep->addChild(unitNode);

        SoQtExaminerViewer::setAccumulationBuffer(true);
        SoQtExaminerViewer::setGLRenderAction(new SoLineHighlightRenderAction);
        SoQtExaminerViewer::setFeedbackVisibility(true);
        SoQtExaminerViewer::setSceneGraph(sceneSep);

        viewAll();
        setAntialiasing(4);
        setBackgroundColor(VirtualRobot::Visualization::Color::None());
    }

    CoinViewer::~CoinViewer()
    {
        sceneSep->removeAllChildren();
        sceneSep->unref();
        unitNode->unref();
    }

    void CoinViewer::addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
    {
        requestLayer(layer).addVisualization(visualization);
        _addVisualization(visualization);
    }

    void CoinViewer::removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
    {
        bool removed = requestLayer(layer).removeVisualization(visualization);
        if (removed)
        {
            _removeVisualization(visualization);
        }
    }

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllVisualizations() const
    {
        std::vector<VirtualRobot::VisualizationPtr> visus;
        for (const auto& entry : layers)
        {
            auto& layerVisus = entry.second.visualizations;
            visus.insert(visus.end(), layerVisus.begin(), layerVisus.end());
        }
        return visus;
    }

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllVisualizations(const std::string &layer) const
    {
        auto it = layers.find(layer);
        if (it != layers.end())
        {
            return std::vector<VirtualRobot::VisualizationPtr>();
        }
        else
        {
            auto& v = it->second.visualizations;
            return std::vector<VirtualRobot::VisualizationPtr>(v.begin(), v.end());
        }
    }

    bool CoinViewer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
    {
        for (const auto& entry : layers)
        {
            if (entry.second.hasVisualization(visualization))
            {
                return true;
            }
        }
        return false;
    }

    bool CoinViewer::hasVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) const
    {
        auto it = layers.find(layer);
        if (it != layers.end())
        {
            return false;
        }
        else
        {
            return it->second.hasVisualization(visualization);
        }
    }

    void CoinViewer::clearLayer(const std::string &layer)
    {
        auto& l = requestLayer(layer);
        for (const auto & v : l.visualizations)
        {
            _removeVisualization(v);
        }
        l.clear();
    }

    bool CoinViewer::hasLayer(const std::string &layer) const
    {
        return layers.find(layer) != layers.end();
    }

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllSelected() const
    {
        static bool printed = false;
        if (!printed)
        {
            VR_ERROR << __FILE__ << " " << __LINE__ << ": NYI" << std::endl;
            printed = true;
        }
    }

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllSelected(const std::string &layer) const
    {
        static bool printed = false;
        if (!printed)
        {
            VR_ERROR << __FILE__ << " " << __LINE__ << ": NYI" << std::endl;
            printed = true;
        }
    }

    QImage CoinViewer::getScreenshot() const
    {
        getSceneManager()->render();
        getSceneManager()->scheduleRedraw();
        QGLWidget* w = (QGLWidget*)getGLWidget();

        QImage i = w->grabFrameBuffer();
        return i;
    }

    void CoinViewer::resetView()
    {
        viewAll();
    }

    void CoinViewer::viewAll()
    {
        SoQtExaminerViewer::viewAll();
    }

    void CoinViewer::setAntialiasing(unsigned short quality)
    {
        if (quality == 0)
        {
            SoQtExaminerViewer::setAntialiasing(false, quality);
        }
        else
        {
            SoQtExaminerViewer::setAntialiasing(true, quality);
        }
    }

    unsigned short CoinViewer::getAntialiasing() const
    {
        SbBool enabled;
        int quality;
        SoQtExaminerViewer::getAntialiasing(enabled, quality);
        return enabled ? static_cast<unsigned short>(quality) : 0;
    }

    void CoinViewer::setBackgroundColor(const VirtualRobot::Visualization::Color &color)
    {
        backgroundColor = color;
        if (color.isNone() || color.isTransparencyOnly())
        {
            SoQtExaminerViewer::setBackgroundColor(SbColor(1, 1, 1));
        }
        else
        {
            SoQtExaminerViewer::setBackgroundColor(SbColor(color.r, color.g, color.b));
        }
    }

    VirtualRobot::Visualization::Color CoinViewer::getBackgroundColor() const
    {
        return backgroundColor;
    }

    CoinViewer::Layer &CoinViewer::requestLayer(const std::string &name)
    {
        auto it = layers.find(name);
        if (it != layers.end())
        {
            return it->second;
        }
        else
        {
            Layer& l = layers[name];
            return l;
        }
    }

    void CoinViewer::_addVisualization(const VirtualRobot::VisualizationPtr &visualization)
    {
        VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
        if (set)
        {
            for (const auto& v : set->getVisualizations())
            {
                _addVisualization(v);
            }
        }
        else
        {
            auto selectionGroup = std::static_pointer_cast<VirtualRobot::CoinSelectionGroup>(visualization->getSelectionGroup());
            auto it = selectionGroups.find(selectionGroup);
            SoSeparator* node = nullptr;
            if (it == selectionGroups.end())
            {
                auto id = selectionGroup->addVisualizationAddedCallback([this](const VirtualRobot::VisualizationPtr& visu)
                {
                    _removeVisualization(visu);
                    _addVisualization(visu);
                });
                SelectionGroupData& d = selectionGroups[selectionGroup];
                d.callbackId = id;
                d.node = new SoSeparator;
                node = d.node;
                node->ref();
                sceneSep->addChild(node);
            }
            else
            {
                node = it->second.node;
            }
            node->addChild(std::static_pointer_cast<VirtualRobot::CoinVisualization>(visualization)->getMainNode());
        }
    }

    void CoinViewer::_removeVisualization(const VirtualRobot::VisualizationPtr &visualization)
    {
        VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
        if (set)
        {
            for (const auto& v : set->getVisualizations())
            {
                _removeVisualization(v);
            }
        }
        else
        {
            auto selectionGroup = std::static_pointer_cast<VirtualRobot::CoinSelectionGroup>(visualization->getSelectionGroup());
            auto it = selectionGroups.find(selectionGroup);
            if (it != selectionGroups.end())
            {
                SelectionGroupData& d = it->second;
                d.node->removeChild(std::static_pointer_cast<VirtualRobot::CoinVisualization>(visualization)->getMainNode());
                if (d.node->getNumChildren() <= 0)
                {
                    sceneSep->removeChild(d.node);
                    d.node->unref();
                    d.node = nullptr;
                    it->first->removeVisualizationAddedCallback(d.callbackId);
                    selectionGroups.erase(it);
                }
            }
            else
            {
                VR_WARNING << "Visualization was not added. ignoring..." << std::endl;
            }
        }
    }

    CoinViewer::Layer::Layer()
    {
    }


    CoinViewer::Layer::~Layer()
    {
    }


    void CoinViewer::Layer::addVisualization(const VirtualRobot::VisualizationPtr &visu)
    {
        if (!hasVisualization(visu))
        {
            visualizations.insert(visu);
        }
    }


    bool CoinViewer::Layer::removeVisualization(const VirtualRobot::VisualizationPtr &visu)
    {
        auto it = visualizations.find(visu);
        if (it != visualizations.end())
        {
            visualizations.erase(it);
            return true;
        }
        return false;
    }


    bool CoinViewer::Layer::hasVisualization(const VirtualRobot::VisualizationPtr &visu) const
    {
        return visualizations.find(visu) != visualizations.end();
    }


    void CoinViewer::Layer::clear()
    {
        visualizations.clear();
    }

}
