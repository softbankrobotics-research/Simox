/*
-----------------------------------------------------------------------------
This source file is part of FRAPPER
research.animationsinstitut.de
sourceforge.net/projects/frapper

Copyright (c) 2008-2009 Filmakademie Baden-Wuerttemberg, Institute of Animation

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; version 2.1 of the License.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
-----------------------------------------------------------------------------
*/

//!
//! \file "OgreTools.h"
//! \brief Header file for OgreTools class.
//!
//! \author     Stefan Habel <stefan.habel@filmakademie.de>
//! \author     Nils Zweiling <nils.zweiling@filmakademie.de>
//! \version    1.0
//! \date       29.06.2009 (last updated)
//!

#ifndef OGRETOOLS_H
#define OGRETOOLS_H

#include "Ogre.h"

namespace VirtualRobot {

//!
//! Class containing camera properties.
//!
class CameraInfo
{

public: // data

    //!
    //! The camera's render width.
    //!
    unsigned int width;

    //!
    //! The camera's render height.
    //!
    unsigned int height;

};


//!
//! Static class with helper functions working on OGRE objects.
//!
class OgreTools
{

public: // static functions

    //!
    //! Clones an Ogre::MovableObject.
    //!
    //! Is needed because OGRE does not provide clone functions for cameras and
    //! lights.
    //!
    //! \param movableObject The object to clone.
    //! \param name The name to use for the object.
    //! \param sceneManager The scene manager to use for creating the object.
    //! \return The cloned object.
    //!
    static Ogre::MovableObject * cloneMovableObject ( Ogre::MovableObject *movableObject, const std::string &name, Ogre::SceneManager *sceneManager = 0 );

    //!
    //! Creates a deep copy of the given scene node.
    //!
    //! \param sceneNode The scene node to copy.
    //! \param sceneNodeCopy The scene node to add the copied objects to (will be created if 0).
    //! \param namePrefix The prefix to use for names of copied objects.
    //! \param sceneManager The scene manager to use for creating the object.
    //!
    static void deepCopySceneNode ( Ogre::SceneNode *sceneNode, Ogre::SceneNode *&sceneNodeCopy, const std::string &namePrefix, Ogre::SceneManager *sceneManager = 0 );

    //!
    //! Creates a copy of the given scene node.
    //!
    //! \param sceneNode The scene node to copy.
    //! \param name The name to use for the copied scene node.
    //! \param sceneManager The scene manager to use for creating the scene node.
    //! \return A copy of the given scene node.
    //!
    static Ogre::SceneNode * copySceneNode ( Ogre::SceneNode *sceneNode, const std::string &name, Ogre::SceneManager *sceneManager = 0 );

    //!
    //! Deletes a whole scene node tree (including attached objects).
    //!
    //! \param sceneNode The scene node containing the tree to delete.
    //! \param sceneManager The scene manager to use for destroying the scene node.
    //! \param deleteRoot Flag to control whether to delete the given scene node.
    //!
    static void deepDeleteSceneNode ( Ogre::SceneNode *sceneNode, Ogre::SceneManager *sceneManager = 0, bool deleteRoot = false );
};

}

#endif
