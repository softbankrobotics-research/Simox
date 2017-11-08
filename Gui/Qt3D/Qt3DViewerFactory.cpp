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

#include "Qt3DViewerFactory.h"
#include "Qt3DViewer.h"

namespace SimoxGui
{

    Qt3DViewerFactory::Qt3DViewerFactory()
    {
    }

    Qt3DViewerFactory::~Qt3DViewerFactory()
    {
    }

    ViewerInterfacePtr Qt3DViewerFactory::createViewer(QWidget *parent)
    {
        Qt3DViewerPtr v(new Qt3DViewer(parent));
        return v;
    }

    /**
    * register this class in the super class factory
    */
    ViewerFactory::SubClassRegistry Qt3DViewerFactory::registry(Qt3DViewerFactory::getName(), &Qt3DViewerFactory::createInstance);

    /**
    * \return "qt3d"
    */
    std::string Qt3DViewerFactory::getName()
    {
        return "qt3d";
    }

    /**
    * \return new instance of Qt3DVisualizationFactory
    */
    ViewerFactoryPtr Qt3DViewerFactory::createInstance(void*)
    {
        Qt3DViewerFactoryPtr qt3DFactory(new Qt3DViewerFactory());
        return qt3DFactory;
    }
}
