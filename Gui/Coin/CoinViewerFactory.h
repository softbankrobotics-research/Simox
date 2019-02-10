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
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once


#include "../ViewerFactory.h"

namespace SimoxGui
{
    /*!
        A Coin based implementation of a ViewerFactory.
    */
    class SIMOX_GUI_IMPORT_EXPORT CoinViewerFactory  : public ViewerFactory
    {
        friend class ViewerFactory;
    protected:
        CoinViewerFactory() = default;

    public:
        virtual ~CoinViewerFactory() override = default;

        virtual AbstractViewerPtr createViewer(QWidget *parent = nullptr, const std::shared_ptr<std::recursive_mutex>& m = std::shared_ptr<std::recursive_mutex>(new std::recursive_mutex)) const override;
        CameraConfigurationPtr createCameraConfiguration() const override;
    };

    typedef std::shared_ptr<CoinViewerFactory> CoinViewerFactoryPtr;


} // namespace SimoxGui
