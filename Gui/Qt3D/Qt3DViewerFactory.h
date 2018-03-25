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
* @copyright  2017 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _Gui_Qt3DViewerFactory_h_
#define _Gui_Qt3DViewerFactory_h_


#include "../ViewerFactory.h"

namespace SimoxGui
{
    /*!
        A Qt3D based implementation of a ViewerFactory.
    */
    class SIMOX_GUI_IMPORT_EXPORT Qt3DViewerFactory  : public ViewerFactory
    {
        friend class ViewerFactory;
    protected:
        Qt3DViewerFactory();

    public:
        virtual ~Qt3DViewerFactory() = default;

        virtual ViewerInterfacePtr createViewer(QWidget *parent = nullptr) const override;
    };

    typedef std::shared_ptr<Qt3DViewerFactory> Qt3DViewerFactoryPtr;

}

#endif
