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
* @author     Philipp Schmidt
* @copyright  2018 Philipp Schmidt
*             GNU Lesser General Public License
*
*/

#include "Qt3DViewer.h"

#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DVisualization.h>
#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DVisualizationSet.h>
#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DSelectionGroup.h>
#include <VirtualRobot/Visualization/SelectionManager.h>

#include <QVBoxLayout>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QPointLight>
#include <Qt3DRender/QMultiSampleAntiAliasing>

SimoxGui::Qt3DViewer::Qt3DViewer(QWidget *parent) : Qt3DExtras::Qt3DWindow(), parent(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this->parent);
    QWidget *container = QWidget::createWindowContainer(this);
    layout->addWidget(container);
    this->parent->setLayout(layout);

    this->scene = new Qt3DCore::QEntity;

    //For antialiasing
    Qt3DRender::QRenderStateSet *renderStateSet = new Qt3DRender::QRenderStateSet;

    Qt3DRender::QMultiSampleAntiAliasing *msaa = new Qt3DRender::QMultiSampleAntiAliasing;
    renderStateSet->addRenderState(msaa);
    Qt3DRender::QDepthTest *depthTest = new Qt3DRender::QDepthTest;
    depthTest->setDepthFunction(Qt3DRender::QDepthTest::LessOrEqual);
    renderStateSet->addRenderState(depthTest);

    this->activeFrameGraph()->setParent(renderStateSet);
    this->setActiveFrameGraph(renderStateSet);

    //For screenshots
    this->capture = new Qt3DRender::QRenderCapture;
    this->activeFrameGraph()->setParent(this->capture);
    this->setActiveFrameGraph(this->capture);

    this->camController = new Qt3DExtras::QOrbitCameraController(scene);
    this->camController->setLinearSpeed( 3500.0f );
    this->camController->setLookSpeed( 240.0f );
    this->camController->setCamera(this->camera());

    this->camera()->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 100000.0f);
    this->camera()->setPosition(QVector3D(0, 0, 2000));
    this->camera()->setViewCenter(QVector3D(0, 0, 0));

    auto lightEntity = new Qt3DCore::QEntity(scene);
    auto light = new Qt3DRender::QPointLight(lightEntity);
    light->setIntensity(1.0f);
    lightEntity->addComponent(light);
    auto lightTransform = new Qt3DCore::QTransform(lightEntity);
    lightTransform->setTranslation(this->camera()->position());
    connect(this->camera(), &Qt3DRender::QCamera::positionChanged, lightTransform, &Qt3DCore::QTransform::setTranslation);
    lightEntity->addComponent(lightTransform);

    this->setRootEntity(scene);
    this->setBackgroundColor(VirtualRobot::Visualization::Color(0.8f, 0.8f, 0.8f, 0.0f));
    this->viewAll();
    this->setAntialiasing(4);

    selectionGroupChangedCallbackId = VirtualRobot::SelectionManager::getInstance()->addSelectionGroupChangedCallback([this](const VirtualRobot::VisualizationPtr& visu, const VirtualRobot::SelectionGroupPtr& old, const VirtualRobot::SelectionGroupPtr&)
    {
        if (_removeVisualization(visu, old))
        {
            _addVisualization(visu);
        }
    });
}

SimoxGui::Qt3DViewer::~Qt3DViewer()
{
    VirtualRobot::SelectionManager::getInstance()->removeSelectionGroupChangedCallback(selectionGroupChangedCallbackId);
    removeAllLayer();
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected() const
{
    VR_ERROR_ONCE_NYI;
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected(const std::string &layer, bool recursive) const
{
    VR_ERROR_ONCE_NYI;
}

QImage SimoxGui::Qt3DViewer::getScreenshot() const
{
    Qt3DRender::QRenderCaptureReply *reply = this->capture->requestCapture();

    //Wait for completion or 3000ms timeout event
    QTimer timer;
    timer.setSingleShot(true);
    QEventLoop loop;
    connect(reply, SIGNAL(completed()), &loop, SLOT(quit()));
    connect(&timer, SIGNAL(timeout()), &loop, SLOT(quit()));
    timer.start(3000);
    loop.exec();

    if(timer.isActive())
    {
        QImage result = reply->image();
        delete reply;
        return result;
    }
    else
    {
        VR_ERROR << "Timeout while capturing screenshot!" << std::endl;
        return QImage();
    }
}

void SimoxGui::Qt3DViewer::resetView()
{
    this->viewAll();
}

void SimoxGui::Qt3DViewer::viewAll()
{
    //this->camera()->viewAll();
}

void SimoxGui::Qt3DViewer::setAntialiasing(unsigned short quality)
{

}

unsigned short SimoxGui::Qt3DViewer::getAntialiasing() const
{
    //qt3d does 4 sampling steps
    return 4;
}

void SimoxGui::Qt3DViewer::setBackgroundColor(const VirtualRobot::Visualization::Color &color)
{
    backgroundColor = color;
    if (color.isNone() || color.isTransparencyOnly())
    {
        this->defaultFrameGraph()->setClearColor(QColor(QRgb(0xffffff)));
    }
    else
    {
        this->defaultFrameGraph()->setClearColor(QColor((int)(color.r * 255.0f), (int)(color.g * 255.0f), (int)(color.b * 255.0f)));
    }
}

VirtualRobot::Visualization::Color SimoxGui::Qt3DViewer::getBackgroundColor() const
{
    return backgroundColor;
}

void SimoxGui::Qt3DViewer::_addVisualization(const VirtualRobot::VisualizationPtr &visualization)
{
    VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
    if (set)
    {
        VR_ASSERT(false);
    }
    else
    {
        auto selectionGroup = std::static_pointer_cast<VirtualRobot::Qt3DSelectionGroup>(visualization->getSelectionGroup());
        auto it = selectionGroups.find(selectionGroup);
        Qt3DCore::QNode* node = nullptr;
        if (it == selectionGroups.end())
        {
            auto selectionChangedId = selectionGroup->addSelectionChangedCallbacks([this,selectionGroup](bool selected)
            {
                SelectionGroupData& d = selectionGroups[selectionGroup];
                VR_ERROR_ONCE_NYI;
            });

            SelectionGroupData& d = selectionGroups[selectionGroup];
            d.selectionChangedCallbackId = selectionChangedId;
            d.node = new Qt3DCore::QNode;
            node = d.node;
            node->setParent(scene);
            d.objectCount = 1;
        }
        else
        {
            node = it->second.node;
            it->second.objectCount++;
        }
        std::static_pointer_cast<VirtualRobot::Qt3DVisualization>(visualization)->getEntity()->setParent(node);
    }
}

bool SimoxGui::Qt3DViewer::_removeVisualization(const VirtualRobot::VisualizationPtr &visualization, const VirtualRobot::SelectionGroupPtr &group)
{
    VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visualization);
    if (set)
    {
        VR_ASSERT(false);
    }
    else
    {
        auto selectionGroup = std::static_pointer_cast<VirtualRobot::Qt3DSelectionGroup>(group ? group : visualization->getSelectionGroup());
        auto it = selectionGroups.find(selectionGroup);
        if (it != selectionGroups.end())
        {
            SelectionGroupData& d = it->second;
            std::static_pointer_cast<VirtualRobot::Qt3DVisualization>(visualization)->getEntity()->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
            d.objectCount--;
            if (d.objectCount <= 0)
            {
                d.node->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
                delete d.node;
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
