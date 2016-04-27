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
#ifndef _Gui_OgreViewerFactory_h_
#define _Gui_OgreViewerFactory_h_


#include "../ViewerFactory.h"

namespace SimoxGui
{
    /*!
        A Ogre based implementation of a GuiFactory.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT OgreViewerFactory  : public ViewerFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OgreViewerFactory();
        virtual ~OgreViewerFactory();

        ViewerInterfacePtr createViewer(QWidget *parent = NULL);

    protected:

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static ViewerFactoryPtr createInstance(void*);
    private:
        static SubClassRegistry registry;
    };

    typedef boost::shared_ptr<OgreViewerFactory> OgreViewerFactoryPtr;

} // namespace SimoxGui

#endif
