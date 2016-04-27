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

#include "OrbitCamera.h"

using namespace SimoxGui;

OrbitCamera::OrbitCamera(Ogre::Camera *camera) :
    camera(camera),
    target(NULL),
    orbiting(false),
    zooming(false),
    lastMouseX(0),
    lastMouseY(0)
{
    initialize();
}

void OrbitCamera::setTarget(Ogre::SceneNode *target)
{
    if(this->target != target)
    {
        this->target = target;
        if(target)
        {
            setYawPitchDist(Ogre::Degree(0), Ogre::Degree(0), 1.5);
            camera->setAutoTracking(true, target);
        }
        else
        {
            camera->setAutoTracking(false);
        }
    }
}

void OrbitCamera::setYawPitchDist(Ogre::Radian yaw, Ogre::Radian pitch, Ogre::Real dist)
{
    camera->setPosition(target->_getDerivedPosition());
    camera->setOrientation(target->_getDerivedOrientation());
    camera->yaw(yaw);
    camera->pitch(-pitch);
    camera->moveRelative(Ogre::Vector3(0, 0, dist));
}

void OrbitCamera::injectMouseMove(const QMouseEvent *event)
{
    Ogre::Real dist = (camera->getPosition() - target->_getDerivedPosition()).length();

    int x = event->x() - lastMouseX;
    int y = event->y() - lastMouseY;

    lastMouseX = event->x();
    lastMouseY = event->y();

    if (orbiting)
    {
        camera->setPosition(target->_getDerivedPosition());
        camera->yaw(Ogre::Degree(-x * 0.25f));
        camera->pitch(Ogre::Degree(-y * 0.25f));
        camera->moveRelative(Ogre::Vector3(0, 0, dist));
    }
    else if(zooming)
    {
        camera->moveRelative(Ogre::Vector3(0, 0, y * 0.004f * dist));
    }
}

void OrbitCamera::injectMouseDown(const QMouseEvent *event)
{
    lastMouseX = event->x();
    lastMouseY = event->y();

    if(event->button() == Qt::LeftButton)
    {
        orbiting = true;
    }
    else if(event->button() == Qt::RightButton)
    {
        zooming = true;
    }
}

void OrbitCamera::injectMouseUp(const QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        orbiting = false;
    }
    else if(event->button() == Qt::RightButton)
    {
        zooming = false;
    }
}

void OrbitCamera::initialize()
{
    setTarget(target? target: camera->getSceneManager()->getRootSceneNode());
    camera->setFixedYawAxis(true);
    setYawPitchDist(Ogre::Degree(0), Ogre::Degree(15), 150);
}
