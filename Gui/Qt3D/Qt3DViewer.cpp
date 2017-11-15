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

SimoxGui::Qt3DViewer::Qt3DViewer(QWidget *parent) : Qt3DExtras::Qt3DWindow(), parent(parent)
{
    QVBoxLayout *layout = new QVBoxLayout(parent);
    QWidget *container = QWidget::createWindowContainer(this);
    layout->addWidget(container);
    parent->setLayout(layout);
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
