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
#include <QApplication>

namespace VirtualRobot
{

    OgreVisualizationFactory::OgreVisualizationFactory()
        :renderer(OgreRenderer::getOgreRenderer())
    {
    }


    OgreVisualizationFactory::~OgreVisualizationFactory()
    {
    }

    void OgreVisualizationFactory::init(int &argc, char* argv[], const std::string &appName)
    {
        // Init qt (qapplication singleton)
        QApplication* app = new QApplication(argc, argv);
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
    * \return new instance of OgreVisualizationFactory
    * if it has not already been called.
    */
    boost::shared_ptr<VisualizationFactory> OgreVisualizationFactory::createInstance(void*)
    {
        boost::shared_ptr<OgreVisualizationFactory> OgreFactory(new OgreVisualizationFactory());
        return OgreFactory;
    }


    /**
    * This method creates a VirtualRobot::OgreVisualizationNode from a given \p filename.
    * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
    *
    * \param filename file to load the visualization from.
    * \param boundingBox Use bounding box instead of full model.
    * \return instance of VirtualRobot::OgreVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
    */
    VisualizationNodePtr OgreVisualizationFactory::getVisualizationFromFile(const std::string& filename, bool boundingBox, float scaleX, float scaleY, float scaleZ)
    {
        if (filename.empty())
        {
            std::cerr <<  "No filename given" << std::endl;
            return VisualizationNodePtr();
        }

        // check for STL file (.stl, .stla, .stlb)
        if (filename.length() >= 4)
        {
            std::string ending = filename.substr(filename.length() - 4, 4);
            BaseIO::getLowerCase(ending);

            if (ending == ".stl" || ending == "stla" || ending == "stlb")
            {
                return getVisualizationFromSTLFile(filename, boundingBox, scaleX, scaleY, scaleZ);
            }
            if (ending == "mesh")
            {
                return getVisualizationFromOgreMeshFile(filename, boundingBox, scaleX, scaleY, scaleZ);
            }
        }

        return getVisualizationFromColladaFile(filename, boundingBox, scaleX, scaleY, scaleZ);
    }

    VisualizationNodePtr OgreVisualizationFactory::getVisualizationFromOgreMeshFile(const std::string& filename, bool boundingBox, float scaleX, float scaleY, float scaleZ)
    {
        if(scaleX != 1.0f || scaleY != 1.0f || scaleZ != 1.0f)
        {
            VR_WARNING << "Scaling not yet supported..." << endl;
        }

        VisualizationNodePtr visualizationNode(new VisualizationNode);

        /*
        // try to open the given file
        SoInput fileInput;

        if (!fileInput.openFile(filename.c_str()))
        {
            std::cerr <<  "Cannot open file " << filename << std::endl;
            return visualizationNode;
        }

        CoinVisualizationFactory::GetVisualizationFromSoInput(fileInput, visualizationNode, boundingBox);

        fileInput.closeFile();
        visualizationNode->setFilename(filename, boundingBox);
*/
        return visualizationNode;
    }

    VisualizationNodePtr OgreVisualizationFactory::getVisualizationFromColladaFile(const std::string& filename, bool boundingBox, float scaleX, float scaleY, float scaleZ)
    {
        if(scaleX != 1.0f || scaleY != 1.0f || scaleZ != 1.0f)
        {
            VR_WARNING << "Scaling not yet supported..." << endl;
        }

        VisualizationNodePtr visualizationNode(new VisualizationNode);

        /*
        // try to open the given file
        SoInput fileInput;

        if (!fileInput.openFile(filename.c_str()))
        {
            std::cerr <<  "Cannot open file " << filename << std::endl;
            return visualizationNode;
        }

        CoinVisualizationFactory::GetVisualizationFromSoInput(fileInput, visualizationNode, boundingBox);

        fileInput.closeFile();
        visualizationNode->setFilename(filename, boundingBox);
*/
        return visualizationNode;
    }


    VirtualRobot::VisualizationNodePtr OgreVisualizationFactory::createBox(float width, float height, float depth, float colorR, float colorG, float colorB)
    {
        if (!renderer)
            return VisualizationNodePtr();
        Ogre::Entity* e = renderer->getSceneManager()->createEntity("Box", Ogre::SceneManager::PT_CUBE);
        Ogre::SceneNode* sn = renderer->getSceneManager()->createSceneNode();
        sn->attachObject(e);
        VirtualRobot::OgreVisualizationNodePtr ovn(new VirtualRobot::OgreVisualizationNode(sn));
        return ovn;
    }

    VisualizationPtr OgreVisualizationFactory::getVisualization(const std::vector<VisualizationNodePtr> &visus)
    {
        boost::shared_ptr<OgreVisualization> v(new OgreVisualization(visus));
        return v;
    }

    VisualizationPtr OgreVisualizationFactory::getVisualization(VisualizationNodePtr visu)
    {
        boost::shared_ptr<OgreVisualization> v(new OgreVisualization(visu));
        return v;
    }

} // namespace VirtualRobot
