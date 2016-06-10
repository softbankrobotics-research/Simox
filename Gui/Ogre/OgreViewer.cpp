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
#include <OGRE/OgreNode.h>
#include <QVBoxLayout>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QApplication>

#ifdef Q_WS_X11
#include <QX11Info>
#endif
#include <VirtualRobot/Visualization/OgreVisualization/OgreVisualization.h>
#include <VirtualRobot/Visualization/OgreVisualization/OgreVisualizationNode.h>

using namespace SimoxGui;

OgreViewer::OgreViewer(QWidget *parent) :
    QWidget(parent),
    renderer(VirtualRobot::OgreRenderer::getOgreRenderer()),
    ogreRoot(NULL),
    ogreWindow(NULL),
    ogreCamera(NULL),
    ogreViewport(NULL),
    ogreSceneManager(NULL),
    cameraController(NULL),
    mUpdatePending(false)
{
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_NoSystemBackground, true);

    QVBoxLayout *layout = new QVBoxLayout(parent);
	layout->addWidget(this);
    parent->setLayout(layout);

    createRenderWindow();

    assert(ogreRoot);
    assert(ogreSceneManager);

    viewerNode = ogreSceneManager->getRootSceneNode()->createChildSceneNode();
}

OgreViewer::~OgreViewer()
{
    if(cameraController)
        delete cameraController;
    if (ogreWindow)
    {
        ogreWindow->removeViewport( 0 );
        ogreWindow->destroy();
    }

    ogreWindow = 0;
    destroy();

    if(ogreRoot)
    {
        ogreRoot->shutdown();
        delete ogreRoot;
    }

    /*if(cameraController)
    {
        delete cameraController;
    }*/

    destroy();
}

void OgreViewer::addLayer(const std::string &layer)
{
    if (hasLayer(layer))
        return;

    layers[layer] = viewerNode->createChildSceneNode(layer);
}


void OgreViewer::addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationPtr &visualization)
{
    /*Ogre::Entity* entity = ogreSceneManager->createEntity("objects/ogre/ogrehead.mesh");
    Ogre::SceneNode* node = ogreSceneManager->getRootSceneNode()->createChildSceneNode();
    node->attachObject(entity);

    cameraController->setTarget(node);*/

    addLayer(layer);
    removeVisualization(layer,id);
    Ogre::SceneNode* ln = layers[layer];
    VirtualRobot::OgreVisualizationPtr ov = boost::dynamic_pointer_cast<VirtualRobot::OgreVisualization>(visualization);
    if (!ov)
        return;
    Ogre::SceneNode* ovisu = ov->getOgreVisualization();
    ln->addChild(ovisu);
}

void OgreViewer::addVisualization(const std::string &layer, const std::string &id, const VirtualRobot::VisualizationNodePtr &visualization)
{
    VirtualRobot::OgreVisualizationNodePtr cvn = boost::dynamic_pointer_cast<VirtualRobot::OgreVisualizationNode>(visualization);
    VirtualRobot::OgreVisualizationPtr ogreVisu(new VirtualRobot::OgreVisualization(cvn));
    addVisualization(layer, id, ogreVisu);
}

void OgreViewer::removeVisualization(const std::string &layer, const std::string &id)
{
    if (!hasLayer(layer))
        return;
    Ogre::SceneNode* ln = layers[layer];
    if (!hasNode(ln, id))
        return;
    ln->removeChild(id);
}

void OgreViewer::clearLayer(const std::string &layer)
{
    if (!hasLayer(layer))
        return;
    Ogre::SceneNode* ln = layers[layer];
    ln->removeAllChildren();
}

bool OgreViewer::hasLayer(const std::string &layer)
{
    if(layers.find(layer) != layers.end())
    {
        // Layer exists
        return true;
    }
    return false;
}

void OgreViewer::setCameraTarget(const VirtualRobot::VisualizationNodePtr &visualization)
{
    VirtualRobot::OgreVisualizationNodePtr ov = boost::dynamic_pointer_cast<VirtualRobot::OgreVisualizationNode>(visualization);
    if (!ov)
        return;
    Ogre::SceneNode* ovisu = ov->getOgreVisualization();
    cameraController->setTarget(ovisu);
    cameraController->setYawPitchDist(Ogre::Degree(0), Ogre::Degree(0), 100.0f);
}

void OgreViewer::start(QWidget *mainWindow)
{
    //QApplication app(0, NULL);
    //mainWindow->show();
    //app.exec();

    QCoreApplication* app = QCoreApplication::instance();
    mainWindow->show();
    app->exec();
}

void OgreViewer::stop()
{

}

void OgreViewer::resetView()
{

}

void OgreViewer::viewAll()
{

}

void OgreViewer::createRenderWindow()
{
    /*resize(width(), height());

    ogreRoot = new Ogre::Root("", "", "");
	//std::string pluginStr(OGRE_RENDERING_PLUGIN);
#ifdef _DEBUG
	std::string pluginStr("RenderSystem_GL_d.dll");
#else
	std::string pluginStr("GL_RenderSystem");
#endif
	ogreRoot->loadPlugin(pluginStr);
    VR_INFO << "Loading rendering plugin: " << OGRE_RENDERING_PLUGIN << std::endl;

    Ogre::RenderSystemList renderers = ogreRoot->getAvailableRenderers();
    std::cout << "Number of renderers found: " << renderers.size() << std::endl;
    assert(!renderers.empty());

    // Choose the first available renderer
    Ogre::RenderSystem *renderer = *renderers.begin();
    assert(renderer);

    ogreRoot->setRenderSystem(renderer);
    ogreRoot->initialise(false);
	Ogre::NameValuePairList params;


	std::string ogreHandle = getOgreHandle();

	params["externalWindowHandle"] = ogreHandle;


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
    ogreWindow->setAutoUpdated(false);*/


    //#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    //    // It is not clear to me why, but having this frame sub-widget
    //    // inside the main widget makes an important difference (under X at
    //    // least).  Without the frame and using this widget's winId()
    //    // below causes trouble when using RenderWidget as a child
    //    // widget.  The frame graphics are completely covered up by the 3D
    //    // render, so using it does not affect the appearance at all.
    //    this->renderFrame = new QFrame;
    //    this->renderFrame->setLineWidth(1);
    //    this->renderFrame->setFrameShadow(QFrame::Sunken);
    //    this->renderFrame->setFrameShape(QFrame::Box);
    //    this->renderFrame->show();

    //    QVBoxLayout *mainLayout = new QVBoxLayout;
    //    mainLayout->setContentsMargins( 0, 0, 0, 0 );
    //    mainLayout->addWidget(this->renderFrame);
    //    this->setLayout(mainLayout);
    //#endif

    //#ifdef Q_OS_MAC
    //    uintptr_t win_id = winId();
    //#else
    //# if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    //    unsigned int win_id = renderFrame->winId();
    //# else
    //    unsigned int win_id = winId();
    //# endif
    //#endif

    intptr_t win_id = (intptr_t)winId();
    QApplication::flush();

    ogreWindow = renderer->createRenderWindow( win_id, width(), height() );
    ogreRoot = renderer->getOgreRoot();

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

void OgreViewer::mouseMoveEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseMove(event);
    }
}

void OgreViewer::mousePressEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseDown(event);
    }
}

void OgreViewer::mouseReleaseEvent(QMouseEvent *event)
{
    if(cameraController)
    {
        cameraController->injectMouseUp(event);
    }
}

QPaintEngine* OgreViewer::paintEngine() const
{
    // This method returns NULL in order to suppress any attempts at
    // painting the underlying widget. The QWidget usually returns
    // a paint engine which causes flickering under Windows.
    return NULL;
}

void OgreViewer::paintEvent(QPaintEvent *pEvent)
{
    update();
}

void OgreViewer::resizeEvent(QResizeEvent *rEvent)
{
    if (rEvent) 
    {
        QWidget::resizeEvent(rEvent);
    }

    if (ogreWindow) 
    {
        ogreWindow->reposition(x(), y());
        ogreWindow->resize(width(), height());
        ogreWindow->windowMovedOrResized();
        if (ogreCamera)
        {
            ogreCamera->setAspectRatio(width() / (float)height());
        }
    }
}

void OgreViewer::update()
{
    QWidget::update();
    ogreRoot->renderOneFrame();
}

void OgreViewer::initializeScene()
{
    Ogre::ResourceGroupManager &rgm = Ogre::ResourceGroupManager::getSingleton();
    rgm.addResourceLocation("./models", "FileSystem");
    rgm.initialiseAllResourceGroups();

    // Note: Use the existing SceneManager because it is used in OgreVisualizationFactory.
    //  SceneNodes created by different SceneManager objects do not cause exceptions or assertions
    //  but are not displayed on the screen either.
    ogreSceneManager = renderer->getSceneManager();
    ogreSceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

    Ogre::Light* light = ogreSceneManager->createLight("MainLight");
    light->setPosition(20, 80, 50);

    ogreCamera = ogreSceneManager->createCamera("Camera");
    ogreCamera->setPosition(Ogre::Vector3(0, 1.5, 0));
    ogreCamera->lookAt(Ogre::Vector3(0, 0, 0));
    ogreCamera->setNearClipDistance(Ogre::Real(0.1));

    ogreViewport = ogreWindow->addViewport(ogreCamera);
    ogreViewport->setBackgroundColour(Ogre::ColourValue(1, 1, 1));

    cameraController = new OrbitCamera(ogreCamera);
}

bool OgreViewer::hasNode(Ogre::SceneNode *sn, const std::string &id)
{
    if (!sn)
        return false;
    Ogre::Node::ChildNodeIterator it = sn->getChildIterator();
    while ( it.hasMoreElements() )
    {
        Ogre::Node *cn = it.getNext();
        if (cn->getName() == id)
            return true;
    }
    return false;
}

