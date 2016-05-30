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

#ifndef _Gui_OgreViewer_h_
#define _Gui_OgreViewer_h_

#include <OGRE/Ogre.h>

#include <QWidget>

#include "../ViewerInterface.h"
#include "OrbitCamera.h"
#include <VirtualRobot/Visualization/OgreVisualization/OgreRenderer.h>

namespace SimoxGui
{

class SIMOX_GUI_IMPORT_EXPORT OgreViewer : public QWindow, public ViewerInterface
{
    Q_OBJECT

    public:
        OgreViewer(QWidget *parent = NULL);
        ~OgreViewer();

        void addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationPtr &visualization);
        void addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationNodePtr &visualization);
        void removeVisualization(const std::string &layer, const std::string &id);

        void clearLayer(const std::string &layer);

        void start(QWidget *mainWindow);
        void stop();

        void resetView();

        void viewAll();

        bool hasLayer(const std::string &layer);

        void setCameraTarget(const VirtualRobot::VisualizationNodePtr &visualization);

    protected:
        void createRenderWindow();

        Ogre::SceneManager *getSceneManager();
        OrbitCamera *getCameraController();

        virtual void mouseMoveEvent(QMouseEvent *event);
        virtual void mousePressEvent(QMouseEvent *event);
        virtual void mouseReleaseEvent(QMouseEvent *event);

        virtual void exposeEvent(QExposeEvent *event);
        virtual bool event(QEvent *event);

        void initializeScene();

    public slots:
        virtual void renderLater();
        virtual void renderNow();

        virtual bool eventFilter(QObject *target, QEvent *event);

    protected:

        void addLayer(const std::string &layer);
        bool hasNode(Ogre::SceneNode *sn, const std::string &id);
        Ogre::Root *ogreRoot;
        Ogre::RenderWindow *ogreWindow;
        Ogre::Camera *ogreCamera;
        Ogre::Viewport *ogreViewport;
        Ogre::SceneManager *ogreSceneManager;

        OrbitCamera *cameraController;

        VirtualRobot::OgreRenderer *renderer;

        Ogre::SceneNode *viewerNode;
        std::map<std::string, Ogre::SceneNode *> layers;
        //std::map<std::string, Ogre::SceneNode *> visualizations;
        //QFrame* renderFrame;

        bool mUpdatePending;
};

    typedef boost::shared_ptr<OgreViewer> OgreViewerPtr;
}

#endif
