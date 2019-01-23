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
* @author     Nikolaus Vahrenkamp, Adrian Knobloch
* @copyright  2016 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include <VirtualRobot/VirtualRobot.h>
#include <string>

#include "AbstractViewer.h"


namespace SimoxGui
{
    class ViewerFactory;
    using ViewerFactoryPtr = std::shared_ptr<ViewerFactory>;

    class CameraConfiguration;
    using CameraConfigurationPtr = std::shared_ptr<CameraConfiguration>;

    class SIMOX_GUI_IMPORT_EXPORT ViewerFactory
    {
    public:
        static ViewerFactoryPtr getInstance();

    protected:
        ViewerFactory() = default;

    public:
        virtual ~ViewerFactory() = default;

        virtual AbstractViewerPtr createViewer(QWidget *parent = nullptr, const std::shared_ptr<std::recursive_mutex>& m = nullptr) const = 0;
        virtual CameraConfigurationPtr createCameraConfiguration() const = 0;
    };

} // namespace SimoxGui
