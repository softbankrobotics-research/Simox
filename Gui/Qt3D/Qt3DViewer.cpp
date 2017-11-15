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

#include <QApplication>
#include <QVBoxLayout>

#include <Qt3DRender/QCamera>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QMaterial>
#include <Qt3DExtras/QTorusMesh>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QOrbitCameraController>


SimoxGui::Qt3DViewer::Qt3DViewer(QWidget *parent) : Qt3DExtras::Qt3DWindow(), parent(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(parent);
    QWidget *container = QWidget::createWindowContainer(this);
    layout->addWidget(container);
    parent->setLayout(layout);

    Qt3DCore::QEntity* scene = new Qt3DCore::QEntity;

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(scene);
    camController->setLinearSpeed( 50.0f );
    camController->setLookSpeed( 180.0f );
    camController->setCamera(this->camera());

    this->camera()->lens()->setPerspectiveProjection(45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
    this->camera()->setPosition(QVector3D(0, 0, 20));
    this->camera()->setViewCenter(QVector3D(0, 0, 0));

    Qt3DCore::QEntity* torus = new Qt3DCore::QEntity(scene);
    Qt3DExtras::QTorusMesh* mesh = new Qt3DExtras::QTorusMesh;
    mesh->setRadius(5);
    mesh->setMinorRadius(1);
    mesh->setRings(100);
    mesh->setSlices(20);

    Qt3DCore::QTransform* transform = new Qt3DCore::QTransform;
    //transform->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 45.f));
    QPropertyAnimation *rotateTransformAnimation = new QPropertyAnimation(transform);
    rotateTransformAnimation->setTargetObject(transform);
    rotateTransformAnimation->setPropertyName("rotationX");
    rotateTransformAnimation->setStartValue(QVariant::fromValue(0));
    rotateTransformAnimation->setEndValue(QVariant::fromValue(360));
    rotateTransformAnimation->setDuration(10000);
    rotateTransformAnimation->setLoopCount(-1);
    rotateTransformAnimation->start();

    Qt3DRender::QMaterial* material = new Qt3DExtras::QPhongMaterial(scene);

    torus->addComponent(mesh);
    torus->addComponent(transform);
    torus->addComponent(material);

    setRootEntity(scene);
}

SimoxGui::Qt3DViewer::~Qt3DViewer()
{
}

void SimoxGui::Qt3DViewer::addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
}

void SimoxGui::Qt3DViewer::removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization)
{
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllVisualizations() const
{
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::Qt3DViewer::getAllVisualizations(const std::string &layer) const
{
}

bool SimoxGui::Qt3DViewer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
{
}

bool SimoxGui::Qt3DViewer::hasVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) const
{
}

void SimoxGui::Qt3DViewer::clearLayer(const std::string &layer)
{
}

bool SimoxGui::Qt3DViewer::hasLayer(const std::string &layer) const
{
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
}

void SimoxGui::Qt3DViewer::setAntialiasing(unsigned short quality)
{
}

unsigned short SimoxGui::Qt3DViewer::getAntialiasing() const
{
}

void SimoxGui::Qt3DViewer::setBackgroundColor(const VirtualRobot::Visualization::Color &color)
{
}

VirtualRobot::Visualization::Color SimoxGui::Qt3DViewer::getBackgroundColor() const
{
}
