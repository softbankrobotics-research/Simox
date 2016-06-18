/*!
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
#ifndef _VirtualRobot_OgreVisualizationFactory_h_
#define _VirtualRobot_OgreVisualizationFactory_h_


#include "../../VirtualRobot.h"
#include "../VisualizationFactory.h"
#include "../../BoundingBox.h"
#include "../../SceneObject.h"
#include "../../EndEffector/EndEffector.h"
#include "../ColorMap.h"
#include "../../Workspace/WorkspaceRepresentation.h"
#include "OgreRenderer.h"

#include <string>
#include <fstream>

namespace VirtualRobot
{
    class VisualizationNode;

    /*!
        A Ogre based implementation of a VisualizationFactory.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT OgreVisualizationFactory  : public VisualizationFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OgreVisualizationFactory();
        virtual ~OgreVisualizationFactory();

        /*!
            Init
        */
        virtual void init(int &argc, char* argv[], const std::string &appName);

        virtual VisualizationNodePtr getVisualizationFromFile(const std::string& filename, bool boundingBox = false, float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f);
        virtual VisualizationNodePtr getVisualizationFromOgreMeshFile(const std::string& filename, bool boundingBox = false, float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f);
        virtual VisualizationNodePtr getVisualizationFromColladaFile(const std::string& filename, bool boundingBox = false, float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f);

        virtual VisualizationNodePtr createBox(float width, float height, float depth, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);

        virtual VisualizationPtr getVisualization(const std::vector<VisualizationNodePtr> &visus);
        virtual VisualizationPtr getVisualization(VisualizationNodePtr visu);

        virtual VisualizationNodePtr getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox = false, Color color = Color::Gray());

        Ogre::SceneNode* createOgreBox(float width, float height, float depth, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        Ogre::SceneNode *createOgreSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);

    protected:
        Ogre::SceneNode* getNodeFromPrimitive(Primitive::PrimitivePtr primitive, bool boundingBox, Color color, const Eigen::Matrix4f &trafo);



        OgreRenderer *renderer;

        // AbstractFactoryMethod
    public:
        static std::string getName();
        static boost::shared_ptr<VisualizationFactory> createInstance(void*);
    private:
        static SubClassRegistry registry;
    };

    typedef boost::shared_ptr<OgreVisualizationFactory> OgreVisualizationFactoryPtr;


} // namespace VirtualRobot

#endif // _VirtualRobot_OgreVisualizationFactory_h_
