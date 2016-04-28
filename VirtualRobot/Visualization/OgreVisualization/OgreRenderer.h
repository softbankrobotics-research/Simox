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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OgreRenderer_h_
#define _VirtualRobot_OgreRenderer_h_

#include <OGRE/Ogre.h>
#include <stdint.h>
#include "../../VirtualRobot.h"

#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

namespace VirtualRobot
{

// mostly taken from Gazebo/rviz

class VIRTUAL_ROBOT_IMPORT_EXPORT OgreRenderer
{
public:
    // retrieve the ogr renderer singleton
  static OgreRenderer* getOgreRenderer();

  Ogre::RenderWindow* createRenderWindow( intptr_t window_id, unsigned int width, unsigned int height );
  static std::string getOgreHandle(intptr_t window_id);
  static std::string getOgreHandle(QWidget *w);

  Ogre::Root* getOgreRoot() { return ogreRoot; }
  Ogre::SceneManager* getSceneManager() { return ogreSceneManager; }

  // Prepare a scene_manager to render overlays.
  // Needed for Ogre >= 1.9 to use fonts; does nothing for prior versions.
  //void prepareOverlays(Ogre::SceneManager* scene_manager);

  // @brief return OpenGl Version as integer, e.g. 320 for OpenGl 3.20
  int getGlVersion() { return gl_version_; }

  // @brief return GLSL Version as integer, e.g. 150 for GLSL 1.50
  int getGlslVersion() { return glsl_version_; }

  // @brief Disables the use of Anti Aliasing
  static void disableAntiAliasing();

  // @brief Force to use the provided OpenGL version on startup
  static void forceGlVersion( int version );

  // @brief Disable stereo rendering even if supported in HW.
  static void forceNoStereo();

  // @brief True if we can render stereo on this device.
  bool isStereoSupported() { return stereo_supported_; }

  /*!
   * \brief setOgreConfigFile Ste the ogre config file
   * \param configFile Can be a relative file which is accessible through VirtualRobot::DataPath
   */
  static void setOgreConfigFile(const std::string & configFile);
  static void setOgrePluginsFile(const std::string & pluginsFile);
  static void setOgreLogFile(const std::string & logFile);
  // list of (potential) plugin paths
  static std::vector<std::string> getOgrePluginPaths();
  static void addOgrePluginPath(const std::string &p);

protected:
	static std::vector<std::string> ogrePaths;

private:
  OgreRenderer();
  void setupDummyWindowId();
  void loadOgrePlugins();


  // helper for makeRenderWindow()
  Ogre::RenderWindow* tryMakeRenderWindow(const std::string& name,
                                          unsigned int width,
                                          unsigned int height,
                                          const Ogre::NameValuePairList* params,
                                          int max_attempts);

  // Find and configure the render system.
  void setupRenderSystem();
  void setupResources();
  void detectGlVersion();

  static OgreRenderer* instance_;

  // ID for a dummy window of size 1x1, used to keep Ogre happy.
  unsigned long dummy_window_id_;

  Ogre::Root* ogreRoot;
  Ogre::SceneManager *ogreSceneManager;
  //Ogre::OverlaySystem* ogre_overlay_system_;

  int gl_version_;
  int glsl_version_;
  static bool use_anti_aliasing_;
  static int force_gl_version_;
  bool stereo_supported_;
  static bool force_no_stereo_;

  static std::string configFile;
  static std::string pluginsFile;
  static std::string logFile;
};

}

#endif

