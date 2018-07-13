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
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/manips/SoTransformManip.h>
#include <QGLWidget>

#include <iostream>

SoPath* pickFilterCB(void *, const SoPickedPoint* pick)
{
   // See which child of selection got picked
   SoFullPath *p = static_cast<SoFullPath*>(pick->getPath());

   //Make sure we didn't select a manipulator
   for (int i = 0; i < p->getLength(); i++)
   {
       SoNode* n = p->getNode(i);

       if (n->isOfType(SoTransformManip::getClassTypeId()))
       {
           //Path includes a manipulator, so just ignore it
           //returning NULL would lead to a deselection of all objects
           //therefore return an empty path
           return new SoPath;
       }
   }

   int depthSelectionNode=0;
   for (; depthSelectionNode<p->getLength(); ++depthSelectionNode)
   {
       if (p->getNode(depthSelectionNode)->getName() == SbName("selectionNode"))
       {
           break;
       }
   }
   SoNode* n = p->getNode(depthSelectionNode + 1); // get SelectionGroup node
   p->truncate(depthSelectionNode+2);
   return p;
}

void selectionCallback(void*, SoPath* p)
{
    auto n = p->getNode(p->getLength()-1);
    VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(n->getUserData());
    VR_ASSERT(s);
    s->setSelected(true);
}

void deselectionCallback(void*, SoPath* p)
{
    auto n = p->getNode(p->getLength()-1);
    VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(n->getUserData());
    VR_ASSERT(s);
    s->setSelected(false);
}


namespace SimoxGui
{
    CoinViewer::CoinViewer(QWidget *parent)
        : SoQtExaminerViewer(parent, "", true, BUILD_POPUP),
          parent(parent),
          sceneSep(new SoSeparator),
          unitNode(new SoUnits),
          selectionNode(new SoSelection)
    {
        sceneSep->ref();
        sceneSep->setName(SbName("sceneSep"));
        unitNode->ref();
        unitNode->setName(SbName("unitNode"));
        selectionNode->ref();
        selectionNode->setName(SbName("selectionNode"));

        unitNode->units = SoUnits::MILLIMETERS;
        sceneSep->addChild(unitNode);

        selectionNode->setPickFilterCallback(pickFilterCB);
        selectionNode->addSelectionCallback(selectionCallback, NULL);
        selectionNode->addDeselectionCallback(deselectionCallback, NULL);

        sceneSep->addChild(selectionNode);

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
        selectionNode->removeAllChildren();
        sceneSep->unref();
        unitNode->unref();
        selectionNode->unref();
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
        std::vector<VirtualRobot::VisualizationPtr> visus;
        const SoPathList* selectedNodes = selectionNode->getList();
        for (int i=0; i<selectedNodes->getLength(); ++i)
        {
            SoPath* p = reinterpret_cast<SoPath*>(selectedNodes->get(i));
            auto userData = p->getNode(p->getLength()-1)->getUserData();
            VR_ASSERT(userData);
            VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(userData);
            for (const auto& v : s->getVisualizations())
            {
                visus.push_back(v);
            }
        }
        return visus;
    }

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllSelected(const std::string &layer) const
    {
        std::vector<VirtualRobot::VisualizationPtr> visus;
        if (hasLayer(layer))
        {
            auto l = requestLayer(layer);
            const SoPathList* selectedNodes = selectionNode->getList();
            for (int i=0; i<selectedNodes->getLength(); ++i)
            {
                SoPath* p = reinterpret_cast<SoPath*>(selectedNodes->get(i));
                auto userData = p->getNode(p->getLength()-1)->getUserData();
                VR_ASSERT(userData);
                VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(userData);
                for (const auto& v : s->getVisualizations())
                {
                    if (l.hasVisualization(v))
                    {
                        visus.push_back(v);
                    }
                }
            }
        }
        return visus;
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
                auto visuAddedId = selectionGroup->addVisualizationAddedCallback([this](const VirtualRobot::VisualizationPtr& visu)
                {
                    _removeVisualization(visu);
                    _addVisualization(visu);
                });
                auto selectionChangedId = selectionGroup->addSelectionChangedCallbacks([this,selectionGroup](bool selected)
                {
                    SelectionGroupData& d = selectionGroups[selectionGroup];
                    if (selected && !selectionNode->isSelected(d.node))
                    {
                        selectionNode->select(d.node);
                    }
                    else if (!selected && selectionNode->isSelected(d.node))
                    {
                        selectionNode->deselect(d.node);
                    }
                });
                SelectionGroupData& d = selectionGroups[selectionGroup];
                d.visuAddedCallbackId = visuAddedId;
                d.selectionChangedCallbackId = selectionChangedId;
                d.node = new SoSeparator;
                node = d.node;
                node->ref();
                node->setUserData(selectionGroup.get());
                static int i=0;
                std::stringstream ss;
                ss << "GroupNode_" << i++;
                node->setName(SbName(ss.str().c_str()));
                selectionNode->addChild(node);
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
                    selectionNode->removeChild(d.node);
                    d.node->unref();
                    d.node = nullptr;
                    it->first->removeVisualizationAddedCallback(d.visuAddedCallbackId);
                    it->first->removeSelectionChangedCallbacks(d.selectionChangedCallbackId);
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
