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
#include "../ViewerFactory.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationSet.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinSelectionGroup.h>
#include <VirtualRobot/Visualization/SelectionManager.h>
#include <VirtualRobot/Tools/MathTools.h>
#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/manips/SoTransformManip.h>
#include <Inventor/nodes/SoCamera.h>
#include <QGLWidget>

#include <iostream>
#include <chrono>

SoPath* pickFilterCB(void *, const SoPickedPoint* pick)
{
    auto selectionMode = VirtualRobot::SelectionManager::getInstance()->getSelectionMode();
    if (selectionMode == VirtualRobot::SelectionManager::SelectionMode::eNone)
    {
        return NULL;
    }
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
        selectionNode->policy = SoSelection::SHIFT;

        sceneSep->addChild(selectionNode);

        SoQtExaminerViewer::setAccumulationBuffer(true);
        SoQtExaminerViewer::setGLRenderAction(new SoLineHighlightRenderAction);
        SoQtExaminerViewer::setFeedbackVisibility(true);
        SoQtExaminerViewer::setSceneGraph(sceneSep);

        viewAll();
        setAntialiasing(4);
        setBackgroundColor(VirtualRobot::Visualization::Color::None());

        selectionGroupChangedCallbackId = VirtualRobot::SelectionManager::getInstance()->addSelectionGroupChangedCallback([this](const VirtualRobot::VisualizationPtr& visu, const VirtualRobot::SelectionGroupPtr& old, const VirtualRobot::SelectionGroupPtr&)
        {
            if (_removeVisualization(visu, old))
            {
                _addVisualization(visu);
            }
        });
    }

    CoinViewer::~CoinViewer()
    {
        VirtualRobot::SelectionManager::getInstance()->removeSelectionGroupChangedCallback(selectionGroupChangedCallbackId);
        removeAllLayers();
        sceneSep->removeAllChildren();
        selectionNode->removeAllChildren();
        sceneSep->unref();
        unitNode->unref();
        selectionNode->unref();
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

    std::vector<VirtualRobot::VisualizationPtr> CoinViewer::getAllSelected(const std::string &layer, bool recursive) const
    {
        std::vector<VirtualRobot::VisualizationPtr> visus;
        if (hasLayer(layer))
        {
            const SoPathList* selectedNodes = selectionNode->getList();
            for (int i=0; i<selectedNodes->getLength(); ++i)
            {
                SoPath* p = reinterpret_cast<SoPath*>(selectedNodes->get(i));
                auto userData = p->getNode(p->getLength()-1)->getUserData();
                VR_ASSERT(userData);
                VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(userData);
                for (const auto& v : s->getVisualizations())
                {
                    if (hasVisualization(v, layer, recursive))
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

    void CoinViewer::setCameraConfiguration(const CameraConfigurationPtr &c)
    {
        SoQtExaminerViewer::getCamera()->position.setValue(c->pose(0, 3), c->pose(1, 3), c->pose(2, 3));
        VirtualRobot::MathTools::Quaternion q = VirtualRobot::MathTools::eigen4f2quat(c->pose);
        SoQtExaminerViewer::getCamera()->orientation.setValue(q.x, q.y, q.z, q.w);
    }

    CameraConfigurationPtr CoinViewer::getCameraConfiguration() const
    {
        CameraConfigurationPtr c = SimoxGui::ViewerFactory::getInstance()->createCameraConfiguration();
        auto rot = SoQtExaminerViewer::getCamera()->orientation.getValue();
        float x, y, z, w;
        rot.getValue(x, y, z, w);
        c->pose = VirtualRobot::MathTools::quat2eigen4f(x, y, z, w);
        auto pos = SoQtExaminerViewer::getCamera()->position.getValue();
        c->pose(0, 3) = pos[0];
        c->pose(1, 3) = pos[1];
        c->pose(2, 3) = pos[2];
        return c;
    }

    void CoinViewer::_addVisualization(const VirtualRobot::VisualizationPtr &visualization)
    {
        VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
        if (set)
        {
            VR_ASSERT(false);
        }
        else
        {
            auto selectionGroup = std::static_pointer_cast<VirtualRobot::CoinSelectionGroup>(visualization->getSelectionGroup());
            auto it = selectionGroups.find(selectionGroup);
            SoSeparator* node = nullptr;
            if (it == selectionGroups.end())
            {
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

    bool CoinViewer::_removeVisualization(const VirtualRobot::VisualizationPtr &visualization, const VirtualRobot::SelectionGroupPtr &group)
    {
        VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
        if (set)
        {
            VR_ASSERT(false);
        }
        else
        {
            auto selectionGroup = std::static_pointer_cast<VirtualRobot::CoinSelectionGroup>(group ? group : visualization->getSelectionGroup());
            auto it = selectionGroups.find(selectionGroup);
            if (it != selectionGroups.end())
            {
                SelectionGroupData& d = it->second;
                d.node->removeChild(std::static_pointer_cast<VirtualRobot::CoinVisualization>(visualization)->getMainNode());
                if (d.node->getNumChildren() <= 0)
                {
                    if (selectionNode->isSelected(d.node))
                    {
                        selectionNode->deselect(d.node);
                    }
                    selectionNode->removeChild(d.node);
                    d.node->unref();
                    d.node = nullptr;
                    it->first->removeSelectionChangedCallbacks(d.selectionChangedCallbackId);
                    selectionGroups.erase(it);
                }
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    void CoinViewer::actualRedraw()
    {
        // require lock
        auto start = std::chrono::system_clock::now();
        auto l = getScopedLock();
        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        if (elapsed.count() > 50)
        {
            VR_WARNING << " Redraw lock time:" << elapsed.count() << std::endl;
        }

        // Render normal scenegraph.
        SoQtExaminerViewer::actualRedraw();
    }
}
