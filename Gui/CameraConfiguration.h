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
* @author     Adrian Knobloch
* @copyright  2018 Adrian Knobloch
*             GNU Lesser General Public License
*
*/

#pragma once

#include "SimoxGuiImportExport.h"

#include <Eigen/Core>

#include <memory>

namespace SimoxGui
{
class CoinViewerFactory;

class SIMOX_GUI_IMPORT_EXPORT CameraConfiguration
{
    friend class CoinViewerFactory;
protected:
    CameraConfiguration() : pose(Eigen::Matrix4f::Identity()) {}

public:
    Eigen::Matrix4f pose;
};
typedef std::shared_ptr<CameraConfiguration> CameraConfigurationPtr;

}
