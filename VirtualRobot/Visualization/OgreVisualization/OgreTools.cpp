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
//! \file "OgreTools.cpp"
//! \brief Implementation file for OgreTools class.
//!
//! \author     Stefan Habel <stefan.habel@filmakademie.de>
//! \author     Nils Zweiling <nils.zweiling@filmakademie.de>
//! \version    1.0
//! \date       29.06.2009 (last updated)
//!

#include "OgreTools.h"

namespace VirtualRobot {

///
/// Public Static Functions
///


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
Ogre::MovableObject * OgreTools::cloneMovableObject ( Ogre::MovableObject *movableObject, const std::string &name, Ogre::SceneManager *sceneManager /* =  0 */ )
{
    // make sure the given object is valid
    if (!movableObject) {
        //Log::error("The given movable object is invalid.", "OgreTools::cloneMovableObject");
        return 0;
    }

    // make sure a valid scene manager is available
    if (!sceneManager)
        sceneManager = movableObject->_getManager();
    if (!sceneManager) {
        //Log::error("No valid scene manager available.", "OgreTools::cloneMovableObject");
        return 0;
    }

    Ogre::MovableObject *result = 0;
    Ogre::String typeName = movableObject->getMovableType();

    try
    {
        if (typeName == "Entity") {

            // create a copy of the entity
            Ogre::Entity *entity = dynamic_cast<Ogre::Entity *>(movableObject);
            Ogre::Entity *entityCopy = sceneManager->createEntity( name, entity->getMesh()->getName());

            // set the same blend mode on entity copy
            if (entity && entityCopy)
            {
                if (entity->hasSkeleton() && entityCopy->hasSkeleton())
                {
                    Ogre::Skeleton *skeleton = entity->getSkeleton();
                    Ogre::Skeleton *skeletonCopy = entityCopy->getSkeleton();
                    skeletonCopy->setBlendMode(skeleton->getBlendMode());
                }

                // copy all custom parameters and materials (shading: e.g. objectIds)
                int numOfSubentities = entity->getNumSubEntities();
                for (int i = 0; i < numOfSubentities; ++i)
                {
                    // copy material names of sub-entities
                    Ogre::SubEntity *subEntity = entity->getSubEntity(i);
                    Ogre::SubEntity *subEntityCopy = entityCopy->getSubEntity(i);
                    subEntityCopy->setMaterialName(subEntity->getMaterialName());

#if (OGRE_VERSION >= 0x010800)
                    if( subEntity->hasCustomParameter(0) ) // This check is only available in Ogre 1.8+
#endif
                        try
                        {
                            const Ogre::Vector4 customParameter = subEntity->getCustomParameter(0); // may throw an exception if not available
                            subEntityCopy->setCustomParameter(0, subEntity->getCustomParameter(0));
                        }
                        catch (Ogre::Exception &e)
                        {
                            //QString message = QString::fromStdString("Subentity %1 of "+entity->getName()+" does not have a custom parameter 0: "+e.getDescription()).arg(i);
                            //Frapper::Log::debug( message, "OgreTools::cloneMoveableObject");
                        }
                }

                // copy all animation states
                Ogre::AnimationStateSet *animationStateSet = entity->getAllAnimationStates();
                Ogre::AnimationStateSet *animationStateSetCopy  = entityCopy->getAllAnimationStates();

                if (animationStateSet && animationStateSetCopy)
                {
                    Ogre::AnimationStateIterator animationStateIter = animationStateSet->getAnimationStateIterator();
                    Ogre::AnimationStateIterator animationStateCopyIter = animationStateSetCopy->getAnimationStateIterator();
                    while( animationStateIter.hasMoreElements() && animationStateCopyIter.hasMoreElements() )
                    {
                        Ogre::AnimationState *animationState = animationStateIter.getNext();
                        Ogre::AnimationState *animationStateCopy = animationStateCopyIter.getNext();
                        animationStateCopy->copyStateFrom(*animationState);
                    }
                }

                result = dynamic_cast<Ogre::MovableObject *>(entityCopy);
            } // end if (entity && entityCopy)
        }
        else if (typeName == "ManualObject")
        {
            Ogre::ManualObject *manualObj = dynamic_cast<Ogre::ManualObject *>(movableObject);
            Ogre::ManualObject *manualObjCopy = sceneManager->createManualObject(name);

            for (unsigned int i=0; i<manualObj->getNumSections(); ++i) {
                Ogre::ManualObject::ManualObjectSection *section = manualObj->getSection(i);

                Ogre::VertexData *vertexData = section->getRenderOperation()->vertexData;

                const size_t vertexElementCount = vertexData->vertexDeclaration->getElementCount();
                const Ogre::String &materialName = section->getMaterialName();
                const Ogre::RenderOperation::OperationType operationType = section->getRenderOperation()->operationType;
                const Ogre::HardwareVertexBufferSharedPtr bufferPtr = vertexData->vertexBufferBinding->getBuffer(0);

                void* vertexBuffer0Data = bufferPtr->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
                float* vertexBuffer0PosFloat = static_cast<float*>(vertexBuffer0Data);
                Ogre::uchar* vertexBuffer0PosChar = static_cast<Ogre::uchar*>(vertexBuffer0Data);

                manualObjCopy->begin(materialName, operationType);
                switch (vertexElementCount) {
                    case 1: {
                        for(int jf=0; jf<vertexData->vertexCount*bufferPtr->getVertexSize()-12; jf+=3) {
                            manualObjCopy->position(
                                vertexBuffer0PosFloat[jf],
                                vertexBuffer0PosFloat[jf+1],
                                vertexBuffer0PosFloat[jf+2] );
                        }
                        break;
                    }
                    case 2: {
                        for(int j=0, jf=0; j<vertexData->vertexCount*bufferPtr->getVertexSize()-16; j+=16, jf+=4) {
                            manualObjCopy->position(
                                vertexBuffer0PosFloat[jf],
                                vertexBuffer0PosFloat[jf+1],
                                vertexBuffer0PosFloat[jf+2] );
                            manualObjCopy->colour(
                                (float)vertexBuffer0PosChar[j+12]/255.f,
                                (float)vertexBuffer0PosChar[j+13]/255.f,
                                (float)vertexBuffer0PosChar[j+14]/255.f,
                                (float)vertexBuffer0PosChar[j+15]/255.f);
                        }
                        break;
                    }
                    case 3: {
                        for(int j=0, jf=0; j<vertexData->vertexCount*bufferPtr->getVertexSize()-28; j+=28, jf+=7) {
                            manualObjCopy->position(
                                vertexBuffer0PosFloat[jf],
                                vertexBuffer0PosFloat[jf+1],
                                vertexBuffer0PosFloat[jf+2] );
                            manualObjCopy->colour(
                                (float)vertexBuffer0PosChar[j+12]/255.f,
                                (float)vertexBuffer0PosChar[j+13]/255.f,
                                (float)vertexBuffer0PosChar[j+14]/255.f,
                                (float)vertexBuffer0PosChar[j+15]/255.f);
                            manualObjCopy->normal(
                                vertexBuffer0PosChar[jf+4],
                                vertexBuffer0PosChar[jf+5],
                                vertexBuffer0PosChar[jf+6]);
                        }
                        break;
                    }
                    case 4: {
                        for(int j=0, jf=0; j<vertexData->vertexCount*bufferPtr->getVertexSize()-36; j+=36, jf+=9) {
                            manualObjCopy->position(
                                vertexBuffer0PosFloat[jf],
                                vertexBuffer0PosFloat[jf+1],
                                vertexBuffer0PosFloat[jf+2] );
                            manualObjCopy->colour(
                                (float)vertexBuffer0PosChar[j+12]/255.f,
                                (float)vertexBuffer0PosChar[j+13]/255.f,
                                (float)vertexBuffer0PosChar[j+14]/255.f,
                                (float)vertexBuffer0PosChar[j+15]/255.f);
                            manualObjCopy->normal(
                                vertexBuffer0PosChar[jf+4],
                                vertexBuffer0PosChar[jf+5],
                                vertexBuffer0PosChar[jf+6]);
                            manualObjCopy->textureCoord(
                                vertexBuffer0PosChar[jf+7],
                                vertexBuffer0PosChar[jf+8]);
                        }
                        break;
                    }
                    default: {
                        //Log::error(QString("Max supported attributes size is 4"), "OgreTools::cloneMovableObject");
                        break;
                    }
                }  // end switch block
                manualObjCopy->end();
                bufferPtr->unlock();
            }

            result = dynamic_cast<Ogre::MovableObject *>(manualObjCopy);
        }
        else if (typeName == "Light")
        {
            // clone light
            Ogre::Light *light = dynamic_cast<Ogre::Light *>(movableObject);
            Ogre::Light *lightCopy = sceneManager->createLight(name);
            lightCopy->setType(light->getType());
            lightCopy->setDiffuseColour(light->getDiffuseColour());
            lightCopy->setSpecularColour(light->getSpecularColour());
            lightCopy->setAttenuation(light->getAttenuationRange(), light->getAttenuationConstant(), light->getAttenuationLinear(), light->getAttenuationQuadric());
            lightCopy->setPosition(light->getPosition());
            lightCopy->setDirection(light->getDirection());
            if (lightCopy->getType() == Ogre::Light::LT_SPOTLIGHT)
                lightCopy->setSpotlightRange(light->getSpotlightInnerAngle(), light->getSpotlightOuterAngle(), light->getSpotlightFalloff());
            lightCopy->setPowerScale(light->getPowerScale());
            lightCopy->setCastShadows(light->getCastShadows());

            result = dynamic_cast<Ogre::MovableObject *>(lightCopy);
        }
        else if (typeName == "Camera")
        {
            // clone camera
            Ogre::Camera *camera = dynamic_cast<Ogre::Camera *>(movableObject);
            Ogre::Camera *cameraCopy = sceneManager->createCamera(name);
            cameraCopy->setAspectRatio(camera->getAspectRatio());
            cameraCopy->setAutoAspectRatio(camera->getAutoAspectRatio());
            cameraCopy->setCastShadows(camera->getCastsShadows());
            cameraCopy->setDirection(camera->getDirection());
            cameraCopy->setFocalLength(camera->getFocalLength());
            cameraCopy->setFOVy(camera->getFOVy());

            cameraCopy->setLodBias(camera->getLodBias());
            cameraCopy->setNearClipDistance(camera->getNearClipDistance());
            cameraCopy->setFarClipDistance(camera->getFarClipDistance());
            cameraCopy->setOrientation(camera->getOrientation());
            cameraCopy->setPolygonMode(camera->getPolygonMode());
            cameraCopy->setPolygonModeOverrideable(camera->getPolygonModeOverrideable());
            cameraCopy->setPosition(camera->getPosition());
            cameraCopy->setProjectionType(camera->getProjectionType());
            cameraCopy->setQueryFlags(camera->getQueryFlags());
            cameraCopy->setRenderingDistance(camera->getRenderingDistance());
            cameraCopy->setRenderQueueGroup(camera->getRenderQueueGroup());
            cameraCopy->setUseIdentityProjection(camera->getUseIdentityProjection());
            cameraCopy->setUseIdentityView(camera->getUseIdentityView());
            cameraCopy->setUseRenderingDistance(camera->getUseRenderingDistance());
            cameraCopy->setVisibilityFlags(camera->getVisibilityFlags());
            cameraCopy->setVisible(camera->getVisible());
            /*
            //cameraCopy->setWindow(...);
            //cameraCopy->setCustomParameter(0, camera->getCustomParameter(0));
            //cameraCopy->setAutoTracking(...);
            //cameraCopy->setCullingFrustum(camera->getCullingFrustum());
            //cameraCopy->setCustomParameter(...);
            //cameraCopy->setCustomProjectionMatrix(..);
            //cameraCopy->setCustomViewMatrix(..);
            //cameraCopy->setDebugDisplayEnabled(...);
            //cameraCopy->setDefaultQueryFlags(...);
            //cameraCopy->setDefaultVisibilityFlags(...);
            //cameraCopy->setFixedYawAxis(...);
            //Ogre::Real left, right, top, bottom;
            //camera->getFrustumExtents(left, right, top, bottom);
            //cameraCopy->setFrustumExtents(left, right, top, bottom);
            //cameraCopy->setFrustumOffset(camera->getFrustumOffset());
            //cameraCopy->setListener(camera->getListener());
            //cameraCopy->setLodCamera(camera->getLodCamera());
            //cameraCopy->setOrthoWindow(...);
            //cameraCopy->setOrthoWindowHeight(...);
            //cameraCopy->setOrthoWindowWidth(...);
            //cameraCopy->setRenderSystemData(camera->getRenderSystemData());
            //cameraCopy->setUserAny(camera->getUserAny());
            */

            //The following seems to be deprecated so I disabled it.

//            if (!movableObject->getUserAny().isEmpty())
//            {
//                try
//                {
//                    CameraInfo *sourceCameraInfo = Ogre::any_cast<CameraInfo *>(movableObject->getUserAny());
//                    if( sourceCameraInfo ) {
//                        CameraInfo *targetCameraInfo = new CameraInfo();
//                        targetCameraInfo->width = sourceCameraInfo->width;
//                        targetCameraInfo->height = sourceCameraInfo->height;
//                        dynamic_cast<Ogre::MovableObject *>(cameraCopy)->setUserAny(Ogre::Any(targetCameraInfo));
//                    }
//                }
//                catch (Ogre::Exception& e)
//                {
//                    //Frapper::Log::error(QString::fromStdString( "Ogre::any_cast to CameraInfo for camera "+camera->getName()+" failed: "+e.getDescription()), "OgreTools::cloneMoveableObject");
//                }
//            }

            //// Setup connections for instances
            //SceneNode *targetSceneNode = new SceneNode(cameraCopy);
            //((Ogre::MovableObject *)cameraCopy)->setUserAny(Ogre::Any(targetSceneNode));
            //if (!((Ogre::MovableObject *)camera)->getUserAny().isEmpty()) {
            //    SceneNode *sourceSceneNode = Ogre::any_cast<SceneNode *>(((Ogre::MovableObject *)camera)->getUserAny());
            //    if (sourceSceneNode) {
            //        QObject::connect(sourceSceneNode, SIGNAL(sceneNodeUpdated()), targetSceneNode, SLOT(updateSceneNode()));
            //    }
            //}

            result = dynamic_cast<Ogre::MovableObject *>(cameraCopy);
        }
    }
    catch (Ogre::Exception& e)
    {
        //Log::error(QString::fromStdString("Could not clone movable object "+movableObject->getName()+" of type "+typeName+": "+e.getDescription()), "OgreTools::cloneMovableObject");
        return NULL;
    }

    return result;
}


//!
//! Creates a deep copy of the given scene node.
//!
//! \param sceneNode The scene node to copy.
//! \param sceneNodeCopy The scene node to add the copied objects to (will be created if 0).
//! \param namePrefix The prefix to use for names of copied objects.
//! \param sceneManager The scene manager to use for creating the object.
//!
void OgreTools::deepCopySceneNode ( Ogre::SceneNode *sceneNode, Ogre::SceneNode *&sceneNodeCopy, const std::string &namePrefix, Ogre::SceneManager *sceneManager /* = 0 */ )
{
    // make sure the given scene node is valid
    if (!sceneNode) {
        //Log::error("The given scene node is invalid.", "OgreTools::deepCopySceneNode");
        return;
    }

    // make sure a valid scene manager is available
    if (!sceneManager)
        sceneManager = sceneNode->getCreator();
    if (!sceneManager) {
        //Log::error("No valid scene manager available.", "OgreTools::deepCopySceneNode");
        return;
    }

    // create the target scene node if it doesn't exist yet
    if (!sceneNodeCopy) {
        std::string sceneNodeCopyName = namePrefix + "_" + sceneNode->getName().c_str() + "Copy";
        sceneNodeCopy = copySceneNode(sceneNode, sceneNodeCopyName, sceneManager);
        if (!sceneNodeCopy) {
            //Log::error("The scene node could not be copied.", "OgreTools::deepCopySceneNode");
            return;
        }
    }

    // iterate over the list of attached objects
    Ogre::SceneNode::ObjectIterator objectIterator = sceneNode->getAttachedObjectIterator();
    while (objectIterator.hasMoreElements()) {
        Ogre::MovableObject *movableObject = objectIterator.getNext();
        if (movableObject) {
            std::string entityCopyName = namePrefix + "_" + movableObject->getName().c_str() + "Copy";
            Ogre::MovableObject *movableObjectCopy = cloneMovableObject(movableObject, entityCopyName, sceneManager);
            if (movableObjectCopy)
                sceneNodeCopy->attachObject(movableObjectCopy);
        }
    }

    // iterate over the list of child nodes
    Ogre::SceneNode::ChildNodeIterator childNodeIterator = sceneNode->getChildIterator();
    while (childNodeIterator.hasMoreElements() ) {
        Ogre::SceneNode *childSceneNode = (Ogre::SceneNode *) childNodeIterator.getNext();
        std::string childSceneNodeCopyName = namePrefix + "_" + childSceneNode->getName().c_str() + "Copy";
        Ogre::SceneNode *childSceneNodeCopy = copySceneNode(childSceneNode, childSceneNodeCopyName, sceneManager);
        if (childSceneNodeCopy) {
            sceneNodeCopy->addChild(childSceneNodeCopy);
            deepCopySceneNode(childSceneNode, childSceneNodeCopy, namePrefix, sceneManager);
        }
    }
}


//!
//! Creates a copy of the given scene node.
//!
//! \param sceneNode The scene node to copy.
//! \param name The name to use for the copied scene node.
//! \param sceneManager The scene manager to use for creating the scene node.
//! \return A copy of the given scene node.
//!
Ogre::SceneNode * OgreTools::copySceneNode ( Ogre::SceneNode *sceneNode, const std::string &name, Ogre::SceneManager *sceneManager /* = 0 */ )
{
    // make sure the given scene node is valid
    if (!sceneNode) {
        //Log::error("The given scene node is invalid.", "OgreTools::copySceneNode");
        return 0;
    }

    // make sure a valid scene manager is available
    if (!sceneManager)
        sceneManager = sceneNode->getCreator();
    if (!sceneManager) {
        //Log::error("No valid scene manager available.", "OgreTools::copySceneNode");
        return 0;
    }

    // check if a scene node of the given name already exists
    if (sceneManager->hasSceneNode(name)) {
        //Log::error(QString("The scene manager already contains a scene node named \"%1\".").arg(name), "OgreTools::copySceneNode");
        return 0;
    }

    // create the scene node copy
    Ogre::SceneNode *sceneNodeCopy = sceneManager->createSceneNode(name);
    if (!sceneNodeCopy) {
        //Log::error("The scene node copy could not be created.", "OgreTools::copySceneNode");
        return 0;
    }


    // copy parameters from scene node to scene node copy
    //sceneNodeCopy->setAutoTracking(...);
    //sceneNodeCopy->setCustomParameter(...);
    //sceneNodeCopy->setDebugDisplayEnabled(...);
    //sceneNodeCopy->setDirection(...);
    //sceneNodeCopy->setFixedYawAxis(...);
    sceneNodeCopy->setInheritOrientation(sceneNode->getInheritOrientation());
    sceneNodeCopy->setInheritScale(sceneNode->getInheritScale());
    //sceneNodeCopy->setInitialState(...);
    //sceneNodeCopy->setInSceneGraph(...);
    sceneNodeCopy->setListener(sceneNode->getListener());
    sceneNodeCopy->setOrientation(sceneNode->getOrientation());
    //sceneNodeCopy->setParent(...);

#if (OGRE_VERSION < 0x010700)
    sceneNodeCopy->setPolygonModeOverrideable(sceneNode->getPolygonModeOverrideable());
#endif

    sceneNodeCopy->setPosition(sceneNode->getPosition());
    //sceneNodeCopy->setRenderSystemData(...);
    sceneNodeCopy->setScale(sceneNode->getScale());

#if (OGRE_VERSION < 0x010700)
    sceneNodeCopy->setUseIdentityProjection(sceneNode->getUseIdentityProjection());
    sceneNodeCopy->setUseIdentityView(sceneNode->getUseIdentityView());
#endif

    //sceneNodeCopy->getUserAny(...);
    //sceneNodeCopy->setVisible(...);
    return sceneNodeCopy;
}

//!
//! Deletes a whole scene node tree (including attached objects).
//!
//! \param sceneNode The scene node containing the tree to delete.
//! \param sceneManager The scene manager to use for destroying the scene node.
//! \param deleteRoot Flag to control whether to delete the given scene node.
//!
void OgreTools::deepDeleteSceneNode ( Ogre::SceneNode *sceneNode, Ogre::SceneManager *sceneManager /* = 0 */, bool deleteRoot /* = false */ )
{
    // make sure the given scene node is valid
    if (!sceneNode) {
        //Log::error("The given scene node is invalid.", "OgreTools::deepDeleteSceneNode");
        return;
    }

    // make sure a valid scene manager is available
    if (!sceneManager)
        sceneManager = sceneNode->getCreator();
    if (!sceneManager) {
        //Log::error("No valid scene manager available.", "OgreTools::deepDeleteSceneNode");
        return;
    }

    // iterate over the list of attached objects
    Ogre::SceneNode::ObjectIterator objectIterator = sceneNode->getAttachedObjectIterator();
    while (objectIterator.hasMoreElements())
    {
        Ogre::String objectName = "";
        try
        {
            Ogre::MovableObject *movableObject = objectIterator.getNext();
            objectName = movableObject->getName();
            sceneNode->detachObject(movableObject);

            //Disabled, because it seems unnecessary to add even more lines of code.
            //Look into original implementation in case of unexpected behaviour.
            //deleteUserAnyFromMovableObject(movableObject);

            if( dynamic_cast<Ogre::Camera *>(movableObject) )
            {
                sceneManager->destroyCamera(dynamic_cast<Ogre::Camera *>(movableObject));
            }
            else if( dynamic_cast<Ogre::Light *>(movableObject) )
            {
                sceneManager->destroyLight(dynamic_cast<Ogre::Light *>(movableObject));
            }
            else if (movableObject->_getManager())
            {
                movableObject->_getManager()->destroyMovableObject(movableObject);
            }
            else
            {
                sceneManager->destroyMovableObject(movableObject);
            }

        }
        catch ( Ogre::Exception& e)
        {
            //Frapper::Log::error( QString::fromStdString("Deleting object "+objectName+" failed: " + e.getDescription()), "OgreTools::deepDeleteSceneNode");
        }
    }

    // iterate over the list of child nodes
    Ogre::SceneNode::ChildNodeIterator childNodeIterator = sceneNode->getChildIterator();
    while (childNodeIterator.hasMoreElements())
    {
        Ogre::SceneNode *childSceneNode = dynamic_cast<Ogre::SceneNode *>(childNodeIterator.getNext());
        if (childSceneNode) {

            //See comment above
            //deleteUserAnyFromSceneNode(childSceneNode);

            deepDeleteSceneNode(childSceneNode, sceneManager);
        }
    }

    // destroy all child nodes of the given scene node
    sceneNode->removeAndDestroyAllChildren();

    // check if the given scene node should be destroyed as well
    if (deleteRoot) {

        //See comment above
        //deleteUserAnyFromSceneNode(sceneNode);

        sceneManager->destroySceneNode(sceneNode);
        sceneNode = NULL; // Don't use this SN after it was deleted!
    }
 }

} // end namespace Frapper
