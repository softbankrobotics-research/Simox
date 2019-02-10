/*!
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

#include "Qt3DViewerFactory.h"
#include "Qt3DViewer.h"

namespace SimoxGui
{
    AbstractViewerPtr Qt3DViewerFactory::createViewer(QWidget *parent, const std::shared_ptr<std::recursive_mutex>& m) const
    {
        return Qt3DViewerPtr(new Qt3DViewer(parent));
    }
}


SimoxGui::CameraConfigurationPtr SimoxGui::Qt3DViewerFactory::createCameraConfiguration() const
{
}
