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
        virtual VisualizationNodePtr createLine(const Eigen::Vector3f& from, const Eigen::Vector3f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createCylinder(float radius, float height, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        //virtual VisualizationNodePtr createCoordSystem(float scaling = 1.0f, std::string* text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10);
        virtual VisualizationNodePtr createBoundingBox(const BoundingBox& bbox, bool wireFrame = false);
        virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        //virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, Eigen::Matrix4f& pose, float scaleX = 1.0f, float scaleY = 1.0f, float scaleZ = 1.0f);
        //virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f& pose, bool showLines = true);
        virtual VisualizationNodePtr createPlane(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createPlane(const MathTools::Plane& plane, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f);
        virtual VisualizationNodePtr createArrow(const Eigen::Vector3f& n, float length = 50.0f, float width = 2.0f, const Color& color = Color::Gray());
        //virtual VisualizationNodePtr createTrajectory(TrajectoryPtr t, Color colorNode = Color::Blue(), Color colorLine = Color::Gray(), float nodeSize = 15.0f, float lineSize = 4.0f);
        //virtual VisualizationNodePtr createText(const std::string& text, bool billboard = false, float scaling = 1.0f, Color c = Color::Black(), float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f);
        //virtual VisualizationNodePtr createEllipse(float x, float y, float z, bool showAxes = true, float axesHeight = 4.0f, float axesWidth = 8.0f);

        virtual VisualizationPtr getVisualization(const std::vector<VisualizationNodePtr> &visus);
        virtual VisualizationPtr getVisualization(VisualizationNodePtr visu);

        virtual VisualizationNodePtr getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox = false, Color color = Color::Gray());

        Ogre::SceneNode* createOgreBox(float width, float height, float depth, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f,  bool wireframe = false);
        Ogre::SceneNode *createOgreSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f, float transparency = 1.0f, int numRings = 32, int numSegments = 32);

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
