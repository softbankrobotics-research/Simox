
#include <QMetaType>

// This is required for QT_MAC_USE_COCOA to be set
#include <QtCore/qglobal.h>

#ifndef Q_OS_MAC
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>
#ifdef Success
  #undef Success
#endif
#endif

// X.h #defines CursorShape to be "0".  Qt uses CursorShape in normal
// C++ way.  This wasn't an issue until ogre_logging.h (below)
// introduced a #include of <QString>.
#ifdef CursorShape
#undef CursorShape
#endif

//#include <OgreRenderWindow.h>
//#include <OgreSceneManager.h>
//#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
//#include <OgreOverlaySystem.h>
//#endif

#include "OgreRenderer.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <QMessageBox>

namespace VirtualRobot
{

OgreRenderer* OgreRenderer::instance_ = 0;
int OgreRenderer::force_gl_version_ = 0;
bool OgreRenderer::use_anti_aliasing_ = true;
bool OgreRenderer::force_no_stereo_ = false;
std::string OgreRenderer::configFile("config/ogre/ogre.cfg");
std::string OgreRenderer::logFile("config/ogre/ogre.log");
std::string OgreRenderer::pluginsFile("config/ogre/ogre_plugins.cfg");

OgreRenderer* OgreRenderer::getOgreRenderer()
{
  if( instance_ == 0 )
  {
    instance_ = new OgreRenderer();
  }
  return instance_;
}

void OgreRenderer::forceGlVersion( int version )
{
  force_gl_version_ = version;
  VR_INFO << "Forcing OpenGl version " << (float)version / 100.0 << ".";
}

void OgreRenderer::disableAntiAliasing()
{
  use_anti_aliasing_ = false;
  VR_INFO << "Disabling Anti-Aliasing";
}

void OgreRenderer::forceNoStereo()
{
  force_no_stereo_ = true;
  VR_INFO << "Forcing Stereo OFF";
}

void OgreRenderer::setOgreConfigFile(const std::string &f)
{
    configFile = f;
}

void OgreRenderer::setOgrePluginsFile(const std::string &f)
{
    pluginsFile = f;
}

void OgreRenderer::setOgreLogFile(const std::string &f)
{
    logFile = f;
}

OgreRenderer::OgreRenderer() :
//    ogre_overlay_system_(NULL),
    stereo_supported_(false)
{
    setupDummyWindowId();
    std::string configFileAbs = configFile;
    std::string pluginsFileAbs = pluginsFile;
    std::string logFileAbs = logFile;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(configFileAbs);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(pluginsFileAbs);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(logFileAbs);
    ogreRoot = new Ogre::Root(pluginsFileAbs, configFileAbs, logFileAbs);
    //#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
    //  ogre_overlay_system_ = new Ogre::OverlaySystem();
    //#endif
    //  loadOgrePlugins();
    setupRenderSystem();
    ogreRoot->initialise(false);
    createRenderWindow( dummy_window_id_, 1, 1 );
    detectGlVersion();
    //  setupResources();
    //  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    ogreSceneManager = ogreRoot->createSceneManager(Ogre::ST_GENERIC);
}

//void OgreRenderer::prepareOverlays(Ogre::SceneManager* scene_manager)
//{
//#if ((OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR >= 9) || OGRE_VERSION_MAJOR >= 2 )
//  if (ogre_overlay_system_)
//    scene_manager->addRenderQueueListener(ogre_overlay_system_);
//#endif
//}

void OgreRenderer::setupDummyWindowId()
{
#ifdef Q_OS_MAC
  dummy_window_id_ = 0;
#else
  Display *display = XOpenDisplay(0);
  assert( display );

  int screen = DefaultScreen( display );

  int attribList[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16, 
                       GLX_STENCIL_SIZE, 8, None };

  XVisualInfo *visual = glXChooseVisual( display, screen, (int*)attribList );

  dummy_window_id_ = XCreateSimpleWindow( display,
                                          RootWindow( display, screen ),
                                          0, 0, 1, 1, 0, 0, 0 );

  GLXContext context = glXCreateContext( display, visual, NULL, 1 );

  glXMakeCurrent( display, dummy_window_id_, context );
#endif
}

void OgreRenderer::loadOgrePlugins()
{
//  std::string plugin_prefix = get_ogre_plugin_path() + "/";
//#ifdef Q_OS_MAC
//  plugin_prefix += "lib";
//#endif
//  ogreRoot->loadPlugin( plugin_prefix + "OgreRenderer_GL" );
//  ogreRoot->loadPlugin( plugin_prefix + "Plugin_OctreeSceneManager" );
//  ogreRoot->loadPlugin( plugin_prefix + "Plugin_ParticleFX" );
}

void OgreRenderer::detectGlVersion()
{
  if ( force_gl_version_ )
  {
    gl_version_ = force_gl_version_;
  }
  else
  {
    Ogre::RenderSystem *renderSys = ogreRoot->getRenderSystem();
    renderSys->createRenderSystemCapabilities();
    const Ogre::RenderSystemCapabilities* caps = renderSys->getCapabilities();
    int major = caps->getDriverVersion().major;
    int minor = caps->getDriverVersion().minor;
    gl_version_ = major * 100 + minor*10;
  }

  switch ( gl_version_ )
  {
    case 200:
      glsl_version_ = 110;
      break;
    case 210:
      glsl_version_ = 120;
      break;
    case 300:
      glsl_version_ = 130;
      break;
    case 310:
      glsl_version_ = 140;
      break;
    case 320:
      glsl_version_ = 150;
      break;
    default:
      if ( gl_version_ > 320 )
      {
        glsl_version_  = gl_version_;
      }
      else
      {
        glsl_version_ = 0;
      }
      break;
  }
  VR_INFO << "OpenGl version: " << (float)gl_version_ / 100.0 << " (GLSL " << (float)glsl_version_ / 100.0 << ").";
}

void OgreRenderer::setupRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  // Get the list of available renderers.
#if OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
//  rsList = ogreRoot->getAvailableRenderers();
#else
  rsList = &(ogreRoot->getAvailableRenderers());
#endif
   
  // Look for the OpenGL one, which we require.
  renderSys = NULL;
  for( unsigned int i = 0; i < rsList->size(); i++ )
  {
    renderSys = rsList->at( i );
    if( renderSys->getName().compare("OpenGL Rendering Subsystem")== 0 )
    {
      break;
    }
  }

  if( renderSys == NULL )
  {
    throw std::runtime_error( "Could not find the opengl rendering subsystem!\n" );
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen","No");

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy.
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  // Set the Full Screen Anti-Aliasing factor.
  if (use_anti_aliasing_) {
    renderSys->setConfigOption("FSAA", "4");
  }

  ogreRoot->setRenderSystem(renderSys);
}

void OgreRenderer::setupResources()
{

}

// On Intel graphics chips under X11, there sometimes comes a
// BadDrawable error during Ogre render window creation.  It is not
// consistent, happens sometimes but not always.  Underlying problem
// seems to be a driver bug.  My workaround here is to notice when
// that specific BadDrawable error happens on request 136 minor 3
// (which is what the problem looks like when it happens) and just try
// over and over again until it works (or until 100 failures, which
// makes it seem like it is a different bug).
static bool x_baddrawable_error = false;
#ifdef Q_WS_X11
static int (*old_error_handler)( Display*, XErrorEvent* );
int checkBadDrawable( Display* display, XErrorEvent* error )
{
  if( error->error_code == BadDrawable &&
      error->request_code == 136 &&
      error->minor_code == 3 )
  {
    x_baddrawable_error = true;
    return 0;
  }
  else
  {
    // If the error does not exactly match the one from the driver bug,
    // handle it the normal way so we see it.
    return old_error_handler( display, error );
  }
}
#endif // Q_WS_X11

Ogre::RenderWindow* OgreRenderer::createRenderWindow( intptr_t window_id, unsigned int width, unsigned int height )
{
  static int windowCounter = 0; // Every RenderWindow needs a unique name

  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = NULL;

  std::stringstream window_handle_stream;
  window_handle_stream << window_id;

#ifdef Q_OS_MAC
  params["externalWindowHandle"] = window_handle_stream.str();
#else
  params["parentWindowHandle"] = window_handle_stream.str();
#endif

  params["externalGLControl"] = true;

  // Enable antialiasing
  if (use_anti_aliasing_) {
    params["FSAA"] = "4";
  }

// Set the macAPI for Ogre based on the Qt implementation
#ifdef QT_MAC_USE_COCOA
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#else
  params["macAPI"] = "carbon";
#endif

  std::ostringstream stream;
  stream << "OgreWindow(" << windowCounter++ << ")";


  // don't bother trying stereo if Ogre does not support it.
#if !OGRE_STEREO_ENABLE
  force_no_stereo_ = true;
#endif

  // attempt to create a stereo window
  bool is_stereo = false;
  /*if (!force_no_stereo_)
  {
    params["stereoMode"] = "Frame Sequential";
    window = tryMakeRenderWindow( stream.str(), width, height, &params, 100);
    params.erase("stereoMode");

    if (window)
    {
#if OGRE_STEREO_ENABLE
      is_stereo = window->isStereoEnabled();
#endif
      if (!is_stereo)
      {
        // Created a non-stereo window.  Discard it and try again (below)
        // without the stereo parameter.
        ogreRoot->detachRenderTarget(window);
        window->destroy();
        window = NULL;
        stream << "x";
        is_stereo = false;
      }
    }
  }*/

  if ( window == NULL )
  {
    window = tryMakeRenderWindow( stream.str(), width, height, &params, 100);
  }

  if( window == NULL )
  {
    VR_ERROR << "Unable to create the rendering window after 100 tries.";
    assert(false);
  }

  if (window)
  {
    window->setActive(true);
    //window->setVisible(true);
    window->setAutoUpdated(false);
  }

  stereo_supported_ = is_stereo;

  return window;
}

Ogre::RenderWindow* OgreRenderer::tryMakeRenderWindow(
      const std::string& name,
      unsigned int width,
      unsigned int height,
      const Ogre::NameValuePairList* params,
      int max_attempts )
{
  Ogre::RenderWindow *window = NULL;
  int attempts = 0;

#ifdef Q_WS_X11
  old_error_handler = XSetErrorHandler( &checkBadDrawable );
#endif

  while (window == NULL && (attempts++) < max_attempts)
  {
    try
    {
      window = ogreRoot->createRenderWindow( name, width, height, false, params );

      // If the driver bug happened, tell Ogre we are done with that
      // window and then try again.
      if( x_baddrawable_error )
      {
        ogreRoot->detachRenderTarget( window );
        window = NULL;
        x_baddrawable_error = false;
      }
    }
    catch( const std::exception & ex )
    {
      std::cerr << "rviz::OgreRenderer: error creating render window: "
                << ex.what() << std::endl;
      window = NULL;
    }
  }

#ifdef Q_WS_X11
  XSetErrorHandler( old_error_handler );
#endif

  if( window && attempts > 1 )
  {
    VR_INFO << "Created render window after %d attempts.", attempts;
  }

  return window;
}


}
