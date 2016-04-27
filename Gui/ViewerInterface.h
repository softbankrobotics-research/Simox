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
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#ifndef _Gui_ViewerInterface_h_
#define _Gui_ViewerInterface_h_

#include <VirtualRobot/VirtualRobot.h>
#include <QtGui/QtGui>

#include <string>

namespace SimoxGui
{

class ViewerInterface
{
    public:
        virtual void addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationPtr &visualization) = 0;
        virtual void addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationNodePtr &visualization) = 0;
        virtual void removeVisualization(const std::string &layer, const std::string &id) = 0;

        virtual void clearLayer(const std::string &layer) = 0;

        virtual void start(QWidget *mainWindow) = 0;
        virtual void stop() = 0;

        virtual void resetView() = 0;

        virtual void viewAll() = 0;
};
typedef boost::shared_ptr<ViewerInterface> ViewerInterfacePtr;

}

#endif

