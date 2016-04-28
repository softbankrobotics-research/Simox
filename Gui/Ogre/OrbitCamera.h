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

#ifndef _Gui_OrbitCamera_h_
#define _Gui_OrbitCamera_h_


#include <OGRE/Ogre.h>
#include <QMouseEvent>
#include "../ViewerInterface.h"

namespace SimoxGui
{

class SIMOX_GUI_IMPORT_EXPORT OrbitCamera
{
    public:
        OrbitCamera(Ogre::Camera *camera);

        virtual void setTarget(Ogre::SceneNode *target);
        virtual void setYawPitchDist(Ogre::Radian yaw, Ogre::Radian pitch, Ogre::Real dist);

        virtual void injectMouseMove(const QMouseEvent *event);
        virtual void injectMouseDown(const QMouseEvent *event);
        virtual void injectMouseUp(const QMouseEvent *event);

    protected:
        void initialize();

    protected:
        Ogre::Camera *camera;
        Ogre::SceneNode *target;
        bool orbiting;
        bool zooming;

        int lastMouseX;
        int lastMouseY;
};

}

#endif
