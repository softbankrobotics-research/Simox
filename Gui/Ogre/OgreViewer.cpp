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

#include "OgreViewer.h"

#include <QX11Info>

using namespace Gui;

OgreViewer::OgreViewer(QWidget *parent) :
    QWidget(parent),
    ogreRoot(NULL),
    ogreWindow(NULL),
    ogreCamera(NULL),
    ogreViewport(NULL),
    ogreSceneManager(NULL),
    cameraController(NULL)
{
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoBackground);

    QVBoxLayout *layout = new QVBoxLayout(parent);
    layout->addWidget(this);
    parent->setLayout(layout);
}

OgreViewer::~OgreViewer()
{
    if(ogreRoot)
    {
        ogreRoot->shutdown();
        delete ogreRoot;
    }

    if(cameraController)
    {
        delete cameraController;
    }

    destroy();
}

void OgreViewer::addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationPtr &visualization)
{

}

void OgreViewer::removeVisualization(const std::string &layer, const std::string &id)
{

}

void OgreViewer::clearLayer(const std::string &layer)
{

}

void OgreViewer::start(QWidget *mainWindow)
{
    //QApplication app(0, NULL);
    //mainWindow->show();
    //app.exec();
}

void OgreViewer::stop()
{

}

void OgreViewer::resetView()
{

}

void OgreViewer::createRenderWindow()
{
    resize(width(), height());

    ogreRoot = new Ogre::Root("", "", "");
    ogreRoot->loadPlugin(OGRE_RENDERING_PLUGIN);
    VR_INFO << "Loading rendering plugin: " << OGRE_RENDERING_PLUGIN << std::endl;

    Ogre::RenderSystemList renderers = ogreRoot->getAvailableRenderers();
    std::cout << "Number of renderers found: " << renderers.size() << std::endl;
    assert(!renderers.empty());

    // Choose the first available renderer
    Ogre::RenderSystem *renderer = *renderers.begin();
    assert(renderer);

    ogreRoot->setRenderSystem(renderer);
    ogreRoot->initialise(false);

    QX11Info info = x11Info();
    Ogre::String handle = Ogre::StringConverter::toString((unsigned long)(info.display())) + ":";
    handle += Ogre::StringConverter::toString((unsigned int)(info.screen())) + ":";
    handle += Ogre::StringConverter::toString((unsigned long)(dynamic_cast<QWidget*>(parent())->winId()));

    Ogre::NameValuePairList params;
    params["externalWindowHandle"] = handle;

    ogreWindow = ogreRoot->createRenderWindow("QOgreWidget_RenderWindow", width(), height(), false, &params);
    ogreWindow->setActive(true);
    ogreWindow->resize(width(), height());
    ogreWindow->setVisible(true);

    WId id;
    ogreWindow->getCustomAttribute("WINDOW", &id);
    assert(id);

    QWidget::create(id);
    resizeEvent(NULL);

    assert(ogreWindow);

    ogreWindow->setActive(true);
    ogreWindow->setVisible(true);
    ogreWindow->setAutoUpdated(false);

    initializeScene();
}

Ogre::SceneManager *OgreViewer::getSceneManager()
{
    return ogreSceneManager;
}

OrbitCamera *OgreViewer::getCameraController()
{
    return cameraController;
}

void OgreViewer::paintEvent(QPaintEvent *event)
{
    if(ogreWindow)
    {
        ogreRoot->_fireFrameStarted();
        ogreWindow->update();
        ogreRoot->_fireFrameEnded();
    }
    else
    {
        createRenderWindow();
    }

    QWidget::paintEvent(event);
}

void OgreViewer::resizeEvent(QResizeEvent *event)
{
    if(ogreWindow)
    {
        ogreWindow->reposition(x(), y());
        ogreWindow->resize(width(), height());
        ogreWindow->windowMovedOrResized();
    }

    if(ogreCamera)
    {
       ogreCamera->setAspectRatio(width() / (float)height());
    }

    QWidget::resizeEvent(event);
}

void OgreViewer::moveEvent(QMoveEvent *event)
{
    if(ogreWindow)
    {
        ogreWindow->windowMovedOrResized();
        update();
    }

    QWidget::moveEvent(event);
}

void OgreViewer::mouseMoveEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseMove(event);
    }

    update();
}

void OgreViewer::mousePressEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseDown(event);
    }

    update();
}

void OgreViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseUp(event);
    }

    update();
}

void OgreViewer::initializeScene()
{
    Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();
    rgm.addResourceLocation("./models", "FileSystem");
    rgm.initialiseAllResourceGroups();

    ogreSceneManager = ogreRoot->createSceneManager(Ogre::ST_GENERIC);
    ogreSceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

    Ogre::Light* light = ogreSceneManager->createLight("MainLight");
    light->setPosition(20, 80, 50);

    ogreCamera = ogreSceneManager->createCamera("Camera");
    ogreCamera->setPosition(Ogre::Vector3(0, 1.5, 0));
    ogreCamera->lookAt(Ogre::Vector3(0, 0, 0));
    ogreCamera->setNearClipDistance(0.1);

    ogreViewport = ogreWindow->addViewport(ogreCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(1, 1, 1));

    cameraController = new OrbitCamera(ogreCamera);

    resizeEvent(NULL);
}

