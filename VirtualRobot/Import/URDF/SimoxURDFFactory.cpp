#include "SimoxURDFFactory.h"

#include <assert.h>
#include <iostream>
#include <map>
#include <stack>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include "../../Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "../../RobotFactory.h"
#include "../../Model/Nodes/ModelJoint.h"
#include "../../Model/Nodes/ModelJointFixed.h"
#include "../../Model/Nodes/ModelLink.h"
#include "../../Model/Nodes/ModelJointPrismatic.h"
#include "../../Model/Nodes/ModelJointRevolute.h"
#include "../../Tools/RuntimeEnvironment.h"
#include "../../CollisionDetection/CollisionModel.h"


using namespace std;
using namespace urdf;

namespace VirtualRobot
{

    SimoxURDFFactory::SimoxURDFFactory()
    {
        useColModelsIfNoVisuModel = true;
    }
    SimoxURDFFactory::~SimoxURDFFactory()
    {
    }

    RobotPtr SimoxURDFFactory::loadFromFile(const std::string& filename, ModelIO::RobotDescription loadMode)
    {

        RobotPtr result;
        boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile(filename.c_str());

        boost::filesystem::path filenameBaseComplete(filename);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();

        if (!urdf_model)
        {
            VR_ERROR << "Error opening urdf file " << filename << endl;
        }
        else
        {
            result = createRobot(*urdf_model, basePath, loadMode, useColModelsIfNoVisuModel);
        }

        return result;
    }



    std::string SimoxURDFFactory::getFileExtension()
    {
        return std::string("urdf");
    }

    std::string SimoxURDFFactory::getFileFilter()
    {
        return std::string("URDF (*.urdf)");
    }

    /**
     * register this class in the super class factory
     */
    RobotImporterFactory::SubClassRegistry SimoxURDFFactory::registry(SimoxURDFFactory::getName(), &SimoxURDFFactory::createInstance);


    /**
     * \return "SimoxURDF"
     */
    std::string SimoxURDFFactory::getName()
    {
        return "SimoxURDF";
    }


    /**
     * \return new instance of SimoxURDFFactory.
     */
    std::shared_ptr<RobotImporterFactory> SimoxURDFFactory::createInstance(void*)
    {
        std::shared_ptr<SimoxURDFFactory> URDFFactory(new SimoxURDFFactory());
        return URDFFactory;
    }

    VirtualRobot::RobotPtr SimoxURDFFactory::createRobot(const urdf::ModelInterface &urdfModel, const std::string& basePath, ModelIO::RobotDescription loadMode, bool useColModelsIfNoVisuModel)
    {
        std::string robotType = urdfModel.getName();
        std::string robotName = robotType;
        std::vector<VirtualRobot::RobotNodePtr> allNodes;
        std::map< std::string, VirtualRobot::RobotNodePtr> allNodesMap;
        std::map< VirtualRobot::RobotNodePtr, std::vector<std::string> > childrenMap;


        VirtualRobot::RobotPtr robo(new VirtualRobot::Model(robotName, robotType));
        std::map<std::string, boost::shared_ptr<Link> > bodies = urdfModel.links_;
        std::map<std::string, boost::shared_ptr<Joint> > joints = urdfModel.joints_;
        std::map<std::string, boost::shared_ptr<Link> >::iterator itBodies = bodies.begin();

        while (itBodies != bodies.end())
        {
            boost::shared_ptr<Link> body = itBodies->second;
            RobotNodePtr nodeVisual = createBodyNode(robo, body, basePath, loadMode, useColModelsIfNoVisuModel);
            allNodes.push_back(nodeVisual);
            allNodesMap[body->name] = nodeVisual;
            std::vector<std::string> childrenlist;

            for (size_t i = 0; i < body->child_joints.size(); i++)
            {
                childrenlist.push_back(body->child_joints[i]->name);
            }

            // ignore -> joint->children
            //for (size_t i=0;i<body->child_links.size();i++)
            //    childrenlist.push_back(body->child_links[i]->name);
            childrenMap[nodeVisual] = childrenlist;
            itBodies++;
        }

        std::map<std::string, boost::shared_ptr<Joint> >::iterator itJoints = joints.begin();

        while (itJoints != joints.end())
        {
            boost::shared_ptr<Joint> joint = itJoints->second;
            RobotNodePtr nodeJoint = createJointNode(robo, joint);
            allNodes.push_back(nodeJoint);
            allNodesMap[joint->name] = nodeJoint;
            std::vector<std::string> childrenlist;
            childrenlist.push_back(joint->child_link_name);
            childrenMap[nodeJoint] = childrenlist;
            itJoints++;
        }

        std::string rname;
        if (urdfModel.getRoot())
            rname = urdfModel.getRoot()->name;

        RobotNodePtr rootNode = allNodesMap[rname];
        if (!rootNode)
            VR_WARNING << "Could not determine root node with name " << rname;

        VirtualRobot::RobotFactory::initializeRobot(robo, allNodes, childrenMap, rootNode);
        return robo;
    }

    Eigen::Matrix4f SimoxURDFFactory::convertPose(urdf::Pose& p)
    {
        const float scale = 1000.0f; // mm
        Eigen::Matrix4f res;
        res.setIdentity();
        double qx, qy, qz, qw;
        p.rotation.getQuaternion(qx, qy, qz, qw);
        res = MathTools::quat2eigen4f(qx, qy, qz, qw);
        res(0, 3) = p.position.x * scale;
        res(1, 3) = p.position.y * scale;
        res(2, 3) = p.position.z * scale;
        return res;
    }

    std::string SimoxURDFFactory::getFilename(const std::string& f, const string &basePath)
    {
        std::string result = f;

        std::string part1 = result.substr(0, 10);

        if (part1 != "package://")
        {
            // No ROS package structure, just try to find the file in the data directory

            boost::filesystem::path p_base(basePath);
            boost::filesystem::path p_f(f);
            result = (p_base / p_f).string();

            if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(result))
            {
                result = f;
                if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(result))
                {
                    VR_ERROR << "Could not determine absolute path of " << result << endl;
                }
            }
        }
        else
        {
            // ROS Package structure, we have to be more intelligent
            result = result.substr(10, result.length() - 10);

            std::string package_base = basePath;

            if(result.find("_description/") != std::string::npos)
            {
                std::string package_name = result.substr(0, result.find("_description/") + 12);
                package_base = basePath.substr(0, basePath.rfind(package_name));
            }

            boost::filesystem::path p_f(result);
            boost::filesystem::path p_base(package_base);
            result = (p_base / p_f).string();
        }

        return result;
    }

    VirtualRobot::VisualizationNodePtr SimoxURDFFactory::convertVisu(boost::shared_ptr<Geometry> g, urdf::Pose& pose, const std::string& basePath)
    {
        const float scale = 1000.0f; // mm
        VirtualRobot::VisualizationNodePtr res;
        std::shared_ptr<VisualizationFactory> factory = CoinVisualizationFactory::createInstance(NULL);

        if (!g)
        {
            return res;
        }

        switch (g->type)
        {
            case urdf::Geometry::BOX:
            {
                boost::shared_ptr<Box> b = boost::dynamic_pointer_cast<Box>(g);
                res = factory->createBox(b->dim.x * scale, b->dim.y * scale, b->dim.z * scale);
            }
            break;

            case urdf::Geometry::SPHERE:
            {
                boost::shared_ptr<Sphere> s = boost::dynamic_pointer_cast<Sphere>(g);
                res = factory->createSphere(s->radius * scale);
            }
            break;


            case urdf::Geometry::CYLINDER:
            {
                boost::shared_ptr<Cylinder> c = boost::dynamic_pointer_cast<Cylinder>(g);
                res = factory->createCylinder(c->radius * scale, c->length * scale);
                
            }
            break;

            case urdf::Geometry::MESH:
            {
                boost::shared_ptr<Mesh> m = boost::dynamic_pointer_cast<Mesh>(g);
                std::string filename = getFilename(m->filename, basePath);
                res = factory->getVisualizationFromFile(filename, false, m->scale.x, m->scale.y, m->scale.z);
                res->setFilename(filename, false);
            }
            break;

            default:
                VR_WARNING << "urdf::Geometry type nyi..." << endl;
        }

        if (res)
        {
            Eigen::Matrix4f p = convertPose(pose);
            if (g->type == urdf::Geometry::CYLINDER)
            {
				// inventor and urdf differ in the conventions for cylinders
                p = p * MathTools::axisangle2eigen4f(Eigen::Vector3f::UnitX(), M_PI_2);
            }
            factory->applyDisplacement(res, p);
        }

        return res;
    }

    VisualizationNodePtr SimoxURDFFactory::convertVisuArray(std::vector<boost::shared_ptr<urdf::Collision> > visu_array, const string &basePath)
    {
        VirtualRobot::VisualizationNodePtr res;
        std::shared_ptr<VisualizationFactory> factory = CoinVisualizationFactory::createInstance(NULL);

        if (visu_array.size()==0)
        {
            return res;
        }

        std::vector< VisualizationNodePtr > visus;
        for (size_t i=0; i<visu_array.size(); i++)
        {
            VirtualRobot::VisualizationNodePtr v = convertVisu(visu_array[i]->geometry, visu_array[i]->origin, basePath);
            if (v)
            {
                visus.push_back(v);
            }
        }
        res = factory->createUnitedVisualization(visus);
        return res;
    }

    VisualizationNodePtr SimoxURDFFactory::convertVisuArray(std::vector<boost::shared_ptr<urdf::Visual> > visu_array, const string &basePath)
    {
        VirtualRobot::VisualizationNodePtr res;
        std::shared_ptr<VisualizationFactory> factory = CoinVisualizationFactory::createInstance(NULL);

        if (visu_array.size()==0)
        {
            return res;
        }

        std::vector< VisualizationNodePtr > visus;
        for (size_t i=0; i<visu_array.size(); i++)
        {
            VirtualRobot::VisualizationNodePtr v = convertVisu(visu_array[i]->geometry, visu_array[i]->origin, basePath);
            if (v)
            {
                visus.push_back(v);
            }
        }
        res = factory->createUnitedVisualization(visus);
        return res;
    }

    RobotNodePtr SimoxURDFFactory::createBodyNode(RobotPtr robo, boost::shared_ptr<Link> urdfBody, const std::string& basePath, ModelIO::RobotDescription loadMode, bool useColModelsIfNoVisuModel)
    {
        const float scale = 1000.0f; // mm
        RobotNodePtr result;

        if (!urdfBody)
        {
            return result;
        }

        std::string name = urdfBody->name;
        Eigen::Matrix4f preJointTransform = Eigen::Matrix4f::Identity();

        VirtualRobot::VisualizationNodePtr rnVisu;
        VirtualRobot::CollisionModelPtr rnCol;

        if (loadMode == ModelIO::RobotDescription::eFull && (urdfBody->visual || urdfBody->visual_array.size()>1))
        {
            if (urdfBody->visual_array.size() > 1)
            {
                // visual points to first entry in array
                rnVisu = convertVisuArray(urdfBody->visual_array, basePath);
            } else
                rnVisu = convertVisu(urdfBody->visual->geometry, urdfBody->visual->origin, basePath);
        }

        if ((loadMode == ModelIO::RobotDescription::eFull  || loadMode == ModelIO::RobotDescription::eCollisionModel) && (urdfBody->collision || urdfBody->collision_array.size()>1) )
        {
            VisualizationNodePtr v;
            if (urdfBody->collision_array.size() > 1)
            {
                v = convertVisuArray(urdfBody->collision_array, basePath);
            } else
                v = convertVisu(urdfBody->collision->geometry, urdfBody->collision->origin, basePath);

            if (v)
            {
                rnCol.reset(new CollisionModel(v));
            }
        }

        if (useColModelsIfNoVisuModel)
        {
            if (rnCol && rnCol->getVisualization() && (!rnVisu || rnVisu->getNumFaces() == 0))
            {
                rnVisu = rnCol->getVisualization()->clone();
            }
        }

        VirtualRobot::ModelLink::Physics physics;

        if (urdfBody->inertial)
        {
            physics.massKg = urdfBody->inertial->mass;
            physics.inertiaMatrix(0, 0) = urdfBody->inertial->ixx;
            physics.inertiaMatrix(0, 1) = urdfBody->inertial->ixy;
            physics.inertiaMatrix(0, 2) = urdfBody->inertial->ixz;

            physics.inertiaMatrix(1, 0) = urdfBody->inertial->ixy;
            physics.inertiaMatrix(1, 1) = urdfBody->inertial->iyy;
            physics.inertiaMatrix(1, 2) = urdfBody->inertial->iyz;

            physics.inertiaMatrix(2, 0) = urdfBody->inertial->ixz;
            physics.inertiaMatrix(2, 1) = urdfBody->inertial->iyz;
            physics.inertiaMatrix(2, 2) = urdfBody->inertial->izz;

            physics.comLocation = VirtualRobot::ModelLink::Physics::eCustom;
            physics.localCoM = convertPose(urdfBody->inertial->origin).block(0, 3, 3, 1);
        }


        result.reset(new ModelLink(robo, name, preJointTransform, rnVisu, rnCol, physics));

        robo->registerModelNode(result);

        return result;
    }

    RobotNodePtr SimoxURDFFactory::createJointNode(RobotPtr robo, boost::shared_ptr<Joint> urdfJoint)
    {
        const float scale = 1000.0f; // mm
        ModelJointPtr result;

        if (!urdfJoint)
        {
            return result;
        }

        std::string name = urdfJoint->name;

        Eigen::Matrix4f preJointTransform = Eigen::Matrix4f::Identity();
        preJointTransform = convertPose(urdfJoint->parent_to_joint_origin_transform);

        Eigen::Vector3f axis;
        axis(0) = urdfJoint->axis.x;
        axis(1) = urdfJoint->axis.y;
        axis(2) = urdfJoint->axis.z;
        float limitLo = float(-M_PI);
        float limitHi = float(M_PI);

        if (urdfJoint->limits)
        {
            limitLo = urdfJoint->limits->lower;
            limitHi = urdfJoint->limits->upper;
        }

        switch (urdfJoint->type)
        {
        case urdf::Joint::REVOLUTE:
            result.reset(new ModelJointRevolute(robo, name, preJointTransform, limitLo, limitHi, axis, 0));
            break;
        case urdf::Joint::CONTINUOUS:
            result.reset(new ModelJointRevolute(robo, name, preJointTransform, limitLo, limitHi, axis, 0));
            VR_INFO << "Todo: continuous joint setup..." << endl;
            break;

        case urdf::Joint::PRISMATIC:
            result.reset(new ModelJointPrismatic(robo, name, preJointTransform, limitLo, limitHi, axis, 0));
            break;

       case urdf::Joint::FIXED:
            result.reset(new ModelJointFixed(robo, name, preJointTransform));
            break;

        default:
            result.reset(new ModelJointFixed(robo, name, preJointTransform));
            std::cout << std::endl << "RobotNode [" << name << "] has a not implemented joint type: " << urdfJoint->type << std::endl << std::endl;
        }

        if (urdfJoint->limits)
        {
            result->setMaxVelocity(urdfJoint->limits->velocity);
            result->setMaxTorque(urdfJoint->limits->effort);
        }

        robo->registerModelNode(result);
        return result;
    }

    void SimoxURDFFactory::set3DModelMode(bool useColModelsIfNoVisuModel)
    {
        this->useColModelsIfNoVisuModel = useColModelsIfNoVisuModel;
    }

}
