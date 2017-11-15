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
* @author     Peter Kaiser
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#ifndef _SimoxGui_ViewerInterface_h_
#define _SimoxGui_ViewerInterface_h_

#include "SimoxGuiImportExport.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/Visualization.h>

#include <QtGui/QtGui>
#include <QImage>

#include <string>

namespace SimoxGui
{

class SIMOX_GUI_IMPORT_EXPORT ViewerInterface
{
public:
    virtual void addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) = 0;
    virtual void removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) = 0;
    virtual std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations() const = 0;
    virtual std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations(const std::string &layer) const = 0;
    virtual bool hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const = 0;
    virtual bool hasVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) const = 0;

    virtual void clearLayer(const std::string &layer) = 0;
    virtual bool hasLayer(const std::string &layer) const = 0;

    virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected() const = 0;
    virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected(const std::string &layer) const = 0;

    virtual QImage getScreenshot() const = 0;

    virtual void resetView() = 0;
    virtual void viewAll() = 0;

    virtual void setAntialiasing(unsigned short quality) = 0;
    virtual unsigned short getAntialiasing() const = 0;

    virtual void setBackgroundColor(const VirtualRobot::Visualization::Color& color) = 0;
    virtual VirtualRobot::Visualization::Color getBackgroundColor() const = 0;
};
typedef std::shared_ptr<ViewerInterface> ViewerInterfacePtr;

}

#endif

