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

#include <QVBoxLayout>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DRender/QDirectionalLight>
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

    this->camController = new Qt3DCustomCameraController(scene);
    this->camController->setLinearSpeed( 3500.0f );
    this->camController->setLookSpeed( 240.0f );
    this->camController->setCamera(this->camera());

    this->camera()->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 100000.0f);
    this->camera()->setPosition(QVector3D(0, 0, 2000));
    this->camera()->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity *lightEntity1 = new Qt3DCore::QEntity(scene);
    Qt3DRender::QDirectionalLight* light1 = new Qt3DRender::QDirectionalLight(lightEntity1);
    light1->setIntensity(1.0f);
    light1->setWorldDirection(QVector3D(-1.0f, -1.0f, -1.0f));
    lightEntity1->addComponent(light1);

    Qt3DCore::QEntity *lightEntity2 = new Qt3DCore::QEntity(scene);
    Qt3DRender::QDirectionalLight* light2 = new Qt3DRender::QDirectionalLight(lightEntity2);
    light2->setIntensity(1.0f);
    light2->setWorldDirection(QVector3D(1.0f, 1.0f, 1.0f));
    lightEntity2->addComponent(light2);

    this->setRootEntity(scene);
    this->setBackgroundColor(VirtualRobot::Visualization::Color(0.8f, 0.8f, 0.8f, 1.0f));
    this->viewAll();
    this->setAntialiasing(4);
}

SimoxGui::Qt3DViewer::~Qt3DViewer()
{
}

void SimoxGui::Qt3DViewer::addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
    requestLayer(layer).addVisualization(visualization);
    _addVisualization(visualization);
}

void SimoxGui::Qt3DViewer::removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
    bool removed = requestLayer(layer).removeVisualization(visualization);
    if (removed)
    {
        _removeVisualization(visualization);
    }
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllVisualizations() const
{
    std::vector<VirtualRobot::VisualizationPtr> tmpVisualizations;
    for (const auto& entry : layers)
    {
        auto& layerVisualizations = entry.second.visualizations;
        tmpVisualizations.insert(tmpVisualizations.end(), layerVisualizations.begin(), layerVisualizations.end());
    }
    return tmpVisualizations;
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllVisualizations(const std::string &layer) const
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

bool SimoxGui::Qt3DViewer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
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

bool SimoxGui::Qt3DViewer::hasVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) const
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

void SimoxGui::Qt3DViewer::clearLayer(const std::string &layer)
{
    auto l = requestLayer(layer);
    for (const auto & v : l.visualizations)
    {
        _removeVisualization(v);
    }
    l.clear();
}

bool SimoxGui::Qt3DViewer::hasLayer(const std::string &layer) const
{
    return layers.find(layer) != layers.end();
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected() const
{
    VR_ERROR_ONCE_NYI;
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected(const std::string &layer) const
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
    this->camera()->viewAll();
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
        this->defaultFrameGraph()->setClearColor(QColor(color.r, color.g, color.b));
    }
}

VirtualRobot::Visualization::Color SimoxGui::Qt3DViewer::getBackgroundColor() const
{
    return backgroundColor;
}

SimoxGui::Qt3DViewer::Layer &SimoxGui::Qt3DViewer::requestLayer(const std::string &name)
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

void SimoxGui::Qt3DViewer::_addVisualization(const VirtualRobot::VisualizationPtr &visualization)
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
        auto selectionGroup = std::static_pointer_cast<VirtualRobot::Qt3DSelectionGroup>(visualization->getSelectionGroup());
        auto it = selectionGroups.find(selectionGroup);
        if (it == selectionGroups.end())
        {
            auto selectionChangedId = selectionGroup->addSelectionChangedCallbacks([this,selectionGroup](bool selected)
            {
                SelectionGroupData& d = selectionGroups[selectionGroup];
                VR_ERROR_ONCE_NYI;
            });
            SelectionGroupData& d = selectionGroups[selectionGroup];
            d.selectionChangedCallbackId = selectionChangedId;

            selectionGroup->getNode()->setParent(scene);
        }
        // TODO add visualization to filter
    }
}

void SimoxGui::Qt3DViewer::_removeVisualization(const VirtualRobot::VisualizationPtr &visualization)
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
        // TODO remove visualization from filter
    }
}

SimoxGui::Qt3DViewer::Layer::Layer()
{
}

SimoxGui::Qt3DViewer::Layer::~Layer()
{
    this->clear();
}

void SimoxGui::Qt3DViewer::Layer::addVisualization(const VirtualRobot::VisualizationPtr &visualization)
{
    if (!hasVisualization(visualization))
    {
        visualizations.insert(visualization);
    }
}

bool SimoxGui::Qt3DViewer::Layer::removeVisualization(const VirtualRobot::VisualizationPtr &visualization)
{
    auto it = visualizations.find(visualization);
    if (it != visualizations.end())
    {
        visualizations.erase(it);
        return true;
    }
    else
    {
        return false;
    }
}

bool SimoxGui::Qt3DViewer::Layer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
{
    return visualizations.find(visualization) != visualizations.end();
}

void SimoxGui::Qt3DViewer::Layer::clear()
{
    visualizations.clear();
}
