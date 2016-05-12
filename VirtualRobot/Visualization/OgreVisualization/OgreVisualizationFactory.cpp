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
#include <boost/algorithm/string.hpp>

namespace VirtualRobot
{
    OgreVisualizationFactory::OgreVisualizationFactory()
        :renderer(NULL)
    {
		renderer = OgreRenderer::getOgreRenderer();
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
        Ogre::SceneNode* sn = createOgreBox(width, height, depth, colorR, colorG, colorB);
        VirtualRobot::OgreVisualizationNodePtr ovn(new VirtualRobot::OgreVisualizationNode(sn));
        return ovn;
    }

    Ogre::SceneNode *OgreVisualizationFactory::createOgreBox(float width, float height, float depth, float colorR, float colorG, float colorB)
    {
        if (!renderer)
            return NULL;
        Ogre::Entity* e = renderer->getSceneManager()->createEntity("Box", Ogre::SceneManager::PT_CUBE);
        Ogre::SceneNode* sn = renderer->getSceneManager()->createSceneNode();
        sn->attachObject(e);
        return sn;
    }

    Ogre::SceneNode *OgreVisualizationFactory::createOgreSphere(float radius, float colorR, float colorG, float colorB)
    {
        if (!renderer)
            return NULL;
        Ogre::Entity* e = renderer->getSceneManager()->createEntity("Sphere", Ogre::SceneManager::PT_SPHERE);
        Ogre::SceneNode* sn = renderer->getSceneManager()->createSceneNode();
        sn->attachObject(e);
        return sn;
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

    VisualizationNodePtr OgreVisualizationFactory::getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr> &primitives, bool boundingBox, VisualizationFactory::Color color)
    {
        VisualizationNodePtr visualizationNode = VisualizationNodePtr(new VisualizationNode());
        Ogre::SceneNode* sn = renderer->getSceneManager()->createSceneNode();

        Eigen::Matrix4f currentTransform = Eigen::Matrix4f::Identity();

        for (std::vector<Primitive::PrimitivePtr>::const_iterator it = primitives.begin(); it != primitives.end(); it++)
        {
            Primitive::PrimitivePtr p = *it;
            currentTransform *= p->transform;
            Ogre::SceneNode* pNode = getNodeFromPrimitive(p, boundingBox, color, currentTransform);
            sn->addChild(pNode);
        }

        if (boundingBox)
        {
            //Ogre::SceneNode* bboxVisu = CreateBoundingBox(sn, false);
            VR_WARNING << "NYI" << endl;
            delete sn;
            //sn = bboxVisu;
        }

        visualizationNode.reset(new OgreVisualizationNode(sn));
        visualizationNode->primitives = primitives;

        return visualizationNode;
    }

    Ogre::SceneNode* OgreVisualizationFactory::getNodeFromPrimitive(Primitive::PrimitivePtr primitive, bool boundingBox, Color color, const Eigen::Matrix4f &trafo)
    {
        Ogre::SceneNode* ov = renderer->getSceneManager()->createSceneNode();

        // todo
        //SoNode* c = getColorNode(color);
        //coinVisualization->addChild(c);

        if (primitive->type == Primitive::Box::TYPE)
        {
            Primitive::Box* box = boost::dynamic_pointer_cast<Primitive::Box>(primitive).get();
            Ogre::SceneNode* ob = createOgreBox(box->width, box->height, box->depth, color.r, color.g, color.b);
            ov->addChild(ob);
        }
        else if (primitive->type == Primitive::Sphere::TYPE)
        {
            Primitive::Sphere* sphere = boost::dynamic_pointer_cast<Primitive::Sphere>(primitive).get();
            Ogre::SceneNode* ob = createOgreSphere(sphere->radius, color.r, color.g, color.b);
            ov->addChild(ob);
            /*SoSphere* soSphere = new SoSphere;
            soSphere->radius = sphere->radius / 1000.f;
            coinVisualization->addChild(soSphere);*/
        }
        else if (primitive->type == Primitive::Cylinder::TYPE)
        {
           /* Primitive::Cylinder* cylinder = boost::dynamic_pointer_cast<Primitive::Cylinder>(primitive).get();
            SoCylinder* soCylinder = new SoCylinder;
            soCylinder->radius = cylinder->radius / 1000.f;
            soCylinder->height = cylinder->height / 1000.f;
            coinVisualization->addChild(soCylinder);*/
            VR_WARNING << "Cylinder nyi" << endl;
        }

        if (boundingBox && ov)
        {
            VR_WARNING << "bbox nyi" << endl;

            //SoSeparator* bboxVisu = CreateBoundingBox(ov, false);
            //ov->addChild(bboxVisu);
        }

        ov->setPosition(trafo(0,3), trafo(1,3), trafo(2,3));
        MathTools::Quaternion q = MathTools::eigen4f2quat(trafo);
        ov->setOrientation(q.w, q.x, q.y, q.z);

        return ov;
    }

} // namespace VirtualRobot
