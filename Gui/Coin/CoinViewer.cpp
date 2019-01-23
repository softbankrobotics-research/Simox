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
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/draggers/SoTransformerDragger.h>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoTransform.h>
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

       if (n->isOfType(SoTransformerDragger::getClassTypeId()))
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

        SoQtExaminerViewer::setSceneGraph(sceneSep);
        SoQtExaminerViewer::setAccumulationBuffer(true);
        highlightRenderAction = new SoGLHighlightRenderAction(getViewportRegion(), this);
        SoQtExaminerViewer::setGLRenderAction(highlightRenderAction);
        SoQtExaminerViewer::redrawOnSelectionChange(selectionNode);
        SoQtExaminerViewer::setFeedbackVisibility(true);

        viewAll();
        setAntialiasing(4);
        setBackgroundColor(VirtualRobot::Visualization::Color::None());

        setAlphaChannel(true);
        setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);

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
        SoQtExaminerViewer::stopAnimating();

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

    std::vector<VirtualRobot::SelectionGroupPtr> CoinViewer::getAllSelectedGroups() const
    {
        std::vector<VirtualRobot::SelectionGroupPtr> visus;
        const SoPathList* selectedNodes = selectionNode->getList();
        for (int i=0; i<selectedNodes->getLength(); ++i)
        {
            SoPath* p = reinterpret_cast<SoPath*>(selectedNodes->get(i));
            auto userData = p->getNode(p->getLength()-1)->getUserData();
            VR_ASSERT(userData);
            VirtualRobot::SelectionGroup* s = reinterpret_cast<VirtualRobot::SelectionGroup*>(userData);
            if (!s->getVisualizations().empty())
            {
                visus.push_back(s->getVisualizations()[0]->getSelectionGroup());
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
        SoQtExaminerViewer::getCamera()->position.setValue(c->pose(0, 3)/1000.f, c->pose(1, 3)/1000.f, c->pose(2, 3)/1000.f);
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
        c->pose(0, 3) = pos[0]*1000.f;
        c->pose(1, 3) = pos[1]*1000.f;
        c->pose(2, 3) = pos[2]*1000.f;
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
                SelectionGroupData& d = selectionGroups[selectionGroup];
                d.selectionChangedCallbackId = selectionGroup->addSelectionChangedCallbacks([this,selectionGroup](bool selected)
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

                    // Force visualization update
                    selectionNode->touch();
                });
                d.manipulatorSetCallbackId = selectionGroup->addManipulatorSetCallback([this,selectionGroup](const VirtualRobot::SelectionGroupPtr&, VirtualRobot::ManipulatorType t)
                {
                    SelectionGroupData& d = selectionGroups[selectionGroup];
                    setDragger(d, t, selectionGroup);
                });
                d.node = new SoSeparator;
                node = d.node;
                node->ref();
                node->setUserData(selectionGroup.get());
                static int i=0;
                std::stringstream ss;
                ss << "GroupNode_" << i++;
                node->setName(SbName(ss.str().c_str()));
                selectionNode->addChild(node);
                d.dragger = nullptr;

                setDragger(d, selectionGroup->getManipulator(), selectionGroup);
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
                    setDragger(d, VirtualRobot::ManipulatorType::None, selectionGroup);

                    if (selectionNode->isSelected(d.node))
                    {
                        selectionNode->deselect(d.node);
                    }
                    // Force visualization update
                    selectionNode->touch();
                    selectionNode->removeChild(d.node);
                    d.node->unref();
                    d.node = nullptr;
                    if (d.dragger)
                    {
                        d.dragger->unref();
                        d.dragger = nullptr;
                    }
                    it->first->removeSelectionChangedCallbacks(d.selectionChangedCallbackId);
                    it->first->removeManipulatorSetCallback(d.manipulatorSetCallbackId);
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

    void CoinViewer::setDragger(CoinViewer::SelectionGroupData &data, VirtualRobot::ManipulatorType t, const VirtualRobot::SelectionGroupPtr& selectionGroup)
    {
        // delete old dragger
        if (data.dragger)
        {
            if (data.node->findChild(data.dragger) >= 0)
            {
                data.node->removeChild(data.dragger);
            }
            data.dragger->unref();
            data.dragger = nullptr;
        }

        if (t == VirtualRobot::ManipulatorType::None)
        {
            // do nothing
        }
        else if (t == VirtualRobot::ManipulatorType::TranslateRotate ||
                 t == VirtualRobot::ManipulatorType::Translate ||
                 t == VirtualRobot::ManipulatorType::Rotate)
        {
            data.dragger = new SoTransformerDragger;
            data.dragger->ref();

            VirtualRobot::BoundingBox b;
            for (const auto& v : selectionGroup->getVisualizations())
            {
                b.addPoints(v->getBoundingBox().getPoints());
            }
            Eigen::Vector3f vec = b.getMax() - b.getMin();
            Eigen::Vector3f midPoint = b.getMin() + vec/2;
            SbMatrix m = SbMatrix::identity();
            float scale = std::max(vec.x(), std::max(vec.y(), vec.z()))/2 + 10;
            m.setTransform(SbVec3f(midPoint.x(), midPoint.y(), midPoint.z()), SbRotation::identity(), SbVec3f(scale, scale, scale));

            (void) data.dragger->enableValueChangedCallbacks(FALSE);
            data.dragger->setMotionMatrix(m);
            data.dragger->setViewportRegion(getViewportRegion());
            (void) data.dragger->enableValueChangedCallbacks(TRUE);

            data.dragger->addStartCallback(&manipulatorStartCallback, static_cast<void*>(selectionGroup.get()));
            data.dragger->addValueChangedCallback(&manipulatorValueChangedCallback, static_cast<void*>(selectionGroup.get()));
            data.dragger->addFinishCallback(&manipulatorFinishCallback, static_cast<void*>(selectionGroup.get()));

            SoSeparator* nullSep = new SoSeparator;

            //Make all scale knobs disappear
            data.dragger->setPart("scale1", nullSep);
            data.dragger->setPart("scale2", nullSep);
            data.dragger->setPart("scale3", nullSep);
            data.dragger->setPart("scale4", nullSep);
            data.dragger->setPart("scale5", nullSep);
            data.dragger->setPart("scale6", nullSep);
            data.dragger->setPart("scale7", nullSep);
            data.dragger->setPart("scale8", nullSep);


            if (t == VirtualRobot::ManipulatorType::Translate)
            {
                data.dragger->setPart("rotator1", nullSep);
                data.dragger->setPart("rotator2", nullSep);
                data.dragger->setPart("rotator3", nullSep);
                data.dragger->setPart("rotator4", nullSep);
                data.dragger->setPart("rotator5", nullSep);
                data.dragger->setPart("rotator6", nullSep);

            }

            if (t == VirtualRobot::ManipulatorType::Rotate)
            {
                data.dragger->setPart("translator1", nullSep);
                data.dragger->setPart("translator2", nullSep);
                data.dragger->setPart("translator3", nullSep);
                data.dragger->setPart("translator4", nullSep);
                data.dragger->setPart("translator5", nullSep);
                data.dragger->setPart("translator6", nullSep);
            }

            data.node->insertChild(data.dragger, 0);
        }
        else
        {
            VR_ERROR << "Unsupported dragger type" << std::endl;
        }
    }

    void CoinViewer::manipulatorStartCallback(void *userdata, SoDragger *)
    {
        VirtualRobot::SelectionGroup* sl = static_cast<VirtualRobot::SelectionGroup*>(userdata);
        sl->executeManipulationStartedCallbacks();
    }

    void CoinViewer::manipulatorValueChangedCallback(void *userdata, SoDragger *dragger)
    {
        VirtualRobot::SelectionGroup* sl = static_cast<VirtualRobot::SelectionGroup*>(userdata);
        SbMatrix m = dragger->getStartMotionMatrix().inverse() * dragger->getMotionMatrix();
        SbVec3f trans, scale;
        SbRotation rot, scaleOrient;
        m.getTransform(trans, rot, scale, scaleOrient);
        float x, y, z, w;
        rot.getValue(x, y, z, w);

        Eigen::Matrix4f em = VirtualRobot::MathTools::quat2eigen4f(x, y, z, w);
        em.block<3, 1>(0, 3) = Eigen::Vector3f(trans[0], trans[1], trans[2]);
        sl->executeManipulationPoseUpdatedCallback(em);
    }

    void CoinViewer::manipulatorFinishCallback(void *userdata, SoDragger *dragger)
    {
        VirtualRobot::SelectionGroup* sl = static_cast<VirtualRobot::SelectionGroup*>(userdata);
        SbMatrix m = dragger->getStartMotionMatrix().inverse() * dragger->getMotionMatrix();
        SbVec3f trans, scale;
        SbRotation rot, scaleOrient;
        m.getTransform(trans, rot, scale, scaleOrient);
        float x, y, z, w;
        rot.getValue(x, y, z, w);

        Eigen::Matrix4f em = VirtualRobot::MathTools::quat2eigen4f(x, y, z, w);
        em.block<3, 1>(0, 3) = Eigen::Vector3f(trans[0], trans[1], trans[2]);
        sl->executeManipulationFinishedCallbacks(em);
    }
}
