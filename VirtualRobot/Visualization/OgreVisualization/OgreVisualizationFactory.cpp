/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/
#include "OgreVisualizationFactory.h"
#include "../VisualizationNode.h"
#include "OgreVisualizationNode.h"
#include "../../VirtualRobotException.h"
#include "../../RuntimeEnvironment.h"
#include "OgreVisualization.h"
#include "../../Robot.h"
#include "../../Grasping/Grasp.h"
#include "../../Trajectory.h"
#include "../../Grasping/GraspSet.h"
#include "../../SceneObject.h"
#include "../../IK/constraints/TSRConstraint.h"
#include "../../IK/constraints/BalanceConstraint.h"
#include "../../IK/constraints/PoseConstraint.h"
#include "../../IK/SupportPolygon.h"
#include "../TriMeshModel.h"
#include "../../Workspace/Reachability.h"
#include "../../Workspace/WorkspaceGrid.h"
#include "../../XML/BaseIO.h"
#include "../../Import/MeshImport/STLReader.h"

/*
#ifdef WIN32
// gl.h assumes windows.h is already included 
// avoid std::min, std::max errors
#  define NOMINMAX
#  include <winsock2.h>
#  include <windows.h>
#endif
#include <GL/gl.h>
*/
#include <iostream>
#include <algorithm>


namespace VirtualRobot
{

    OgreVisualizationFactory::OgreVisualizationFactory()
    {
    }


    OgreVisualizationFactory::~OgreVisualizationFactory()
    {
    }

    void OgreVisualizationFactory::init(int &argc, char* argv[], const std::string &appName)
    {


    }

    /**
    * register this class in the super class factory
    */
    VisualizationFactory::SubClassRegistry OgreVisualizationFactory::registry(OgreVisualizationFactory::getName(), &OgreVisualizationFactory::createInstance);


    /**
    * \return "ogre"
    */
    std::string OgreVisualizationFactory::getName()
    {
        return "ogre";
    }


    /**
    * \return new instance of OgreVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    boost::shared_ptr<VisualizationFactory> OgreVisualizationFactory::createInstance(void*)
    {
        boost::shared_ptr<OgreVisualizationFactory> OgreFactory(new OgreVisualizationFactory());
        return OgreFactory;
    }

    VirtualRobot::VisualizationNodePtr OgreVisualizationFactory::createBox(float width, float height, float depth, float colorR, float colorG, float colorB)
    {
        // todo !!!
        return VisualizationNodePtr();
    }

} // namespace VirtualRobot
