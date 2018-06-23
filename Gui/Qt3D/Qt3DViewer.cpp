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
* @copyright  2017 Philipp Schmidt
*             GNU Lesser General Public License
*
*/

#include "Qt3DViewer.h"

#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DVisualization.h>
#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DVisualizationSet.h>
#include <VirtualRobot/Visualization/Qt3DVisualization/Qt3DElement.h>


#include <QVBoxLayout>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/qforwardrenderer.h>
#include <Qt3DRender/QDirectionalLight>

SimoxGui::Qt3DViewer::Qt3DViewer(QWidget *parent) : Qt3DExtras::Qt3DWindow(), parent(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(this->parent);
    QWidget *container = QWidget::createWindowContainer(this);
    layout->addWidget(container);
    this->parent->setLayout(layout);

    this->scene = new Qt3DCore::QEntity;

    this->camController = new Qt3DExtras::QOrbitCameraController(scene);
    this->camController->setLinearSpeed( 500.0f );
    this->camController->setLookSpeed( 180.0f );
    this->camController->setCamera(this->camera());

    this->camera()->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 100000.0f);
    this->camera()->setPosition(QVector3D(0, 0, 3000));
    this->camera()->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(scene);

    Qt3DRender::QDirectionalLight* light = new Qt3DRender::QDirectionalLight(lightEntity);
    light->setIntensity(1.0f);
    light->setWorldDirection(QVector3D(-1.0f, -1.0f, -1.0f));

    lightEntity->addComponent(light);

    this->setRootEntity(scene);
    this->setBackgroundColor(VirtualRobot::Visualization::Color(0.8f, 0.8f, 0.8f, 1.0f));
}

SimoxGui::Qt3DViewer::~Qt3DViewer()
{
}

void SimoxGui::Qt3DViewer::addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
    this->scene->dumpObjectTree();
    requestLayer(layer).addVisualization(visualization);
    this->scene->dumpObjectTree();
}

void SimoxGui::Qt3DViewer::removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
    requestLayer(layer).removeVisualization(visualization);
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
    requestLayer(layer).clear();
}

bool SimoxGui::Qt3DViewer::hasLayer(const std::string &layer) const
{
    return layers.find(layer) != layers.end();
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected() const
{
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllSelected(const std::string &layer) const
{
}

QImage SimoxGui::Qt3DViewer::getScreenshot() const
{
}

void SimoxGui::Qt3DViewer::resetView()
{
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
}

void SimoxGui::Qt3DViewer::setBackgroundColor(const VirtualRobot::Visualization::Color &color)
{
    this->defaultFrameGraph()->setClearColor(QColor(QRgb(0x4d4d4f)));
}

VirtualRobot::Visualization::Color SimoxGui::Qt3DViewer::getBackgroundColor() const
{
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
        l.layerMainNode->setParent(this->scene);
        return l;
    }
}

SimoxGui::Qt3DViewer::Layer::Layer() : layerMainNode(new Qt3DCore::QNode)
{

}

SimoxGui::Qt3DViewer::Layer::~Layer()
{
    this->clear();
    delete this->layerMainNode;
}

void SimoxGui::Qt3DViewer::Layer::addVisualization(const VirtualRobot::VisualizationPtr &visualization)
{
    if (!hasVisualization(visualization))
    {
        visualizations.insert(visualization);
        VirtualRobot::Qt3DElement* visu = dynamic_cast<VirtualRobot::Qt3DElement*>(visualization.get());
        if(visu)
        {
            visu->getEntity()->setParent(this->layerMainNode);
        }
    }
}

void SimoxGui::Qt3DViewer::Layer::removeVisualization(const VirtualRobot::VisualizationPtr &visualization)
{
    auto it = visualizations.find(visualization);
    if (it != visualizations.end())
    {
        visualizations.erase(it);
        VirtualRobot::Qt3DElement* visu = dynamic_cast<VirtualRobot::Qt3DElement*>(visualization.get());
        if(visu)
        {
            visu->getEntity()->setParent((Qt3DCore::QNode*) nullptr);
        }
    }
}

bool SimoxGui::Qt3DViewer::Layer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
{
    return visualizations.find(visualization) != visualizations.end();
}

void SimoxGui::Qt3DViewer::Layer::clear()
{
    for(Qt3DCore::QNode* node : layerMainNode->childNodes())
    {
        node->setParent((Qt3DCore::QNode*) nullptr);
    }
    visualizations.clear();
}
