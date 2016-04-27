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
#ifndef _VirtualRobot_OgreVisualization_h_
#define _VirtualRobot_OgreVisualization_h_

#include "../../VirtualRobot.h"
#include "../Visualization.h"
#include "OgreRenderer.h"


class SoNode;

namespace VirtualRobot
{

    /*!
        A Ogre3D based implementation of a visualization.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT OgreVisualization : public Visualization
    {
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OgreVisualization(const VisualizationNodePtr visualizationNode);
        OgreVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes);
        virtual ~OgreVisualization();

        virtual bool highlight(VisualizationNodePtr visualizationNode, bool enable);
        virtual bool highlight(unsigned int which, bool enable);

        virtual void colorize(VisualizationFactory::Color c);
        virtual void setTransparency(float transparency);

        virtual VisualizationPtr clone();

        Ogre::SceneNode* getOgreVisualization();

        static std::string getFactoryName()
        {
            return "ogre";
        }

    protected:
        bool buildVisualization();

        OgreRenderer* ogreRenderer;

        Ogre::SceneNode* sceneNode;

        // todo !!!
        void* color;
    };

    typedef boost::shared_ptr<OgreVisualization> OgreVisualizationPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_OgreVisualization_h_
