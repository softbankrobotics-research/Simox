
#include "Obstacle.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "Nodes/ModelLink.h"
#include "../Visualization/VisualizationFactory.h"
#include "../VirtualRobotException.h"
#include "../Visualization/TriMeshModel.h"

#include <vector>

namespace VirtualRobot
{

    // obstacle models start with 20000
    //int Obstacle::idCounter = 20000;


    Obstacle::Obstacle(const std::string& name)
        : Model(name, "Obstacle")
    {
    }

    Obstacle::~Obstacle()
    {
    }

    VirtualRobot::ObstaclePtr Obstacle::create(const std::string& name, const VisualizationPtr& visualization, const CollisionModelPtr& collisionModel, const ModelLink::Physics& p, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr m(new Obstacle(name));
        ModelLinkPtr node(new ModelLink(m,
            name,
            Eigen::Matrix4f::Identity(),
            visualization,
            collisionModel,
            p,
            colChecker ? colChecker : CollisionChecker::getGlobalCollisionChecker()));

        m->registerModelNode(node);
        m->setRootNode(node);
        m->setGlobalPose(Eigen::Matrix4f::Identity());

        return m;
    }

    VirtualRobot::ObstaclePtr Obstacle::createBox(float width, float height, float depth, Visualization::Color color, const std::string& visualizationType, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }

        /*
        std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Box(width,height,depth));
        primitives.push_back(p);

        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        VisualizationPtr visu = visualizationFactory->createBox(width, height, depth);
        visu->setColor(color);

        if (!visu)
        {
            VR_ERROR << "Could not create box visualization with visu type " << visualizationType << endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        //int id = idCounter;
        //idCounter++;

        std::stringstream ss;
        ss << "Box";

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker));
        result = Obstacle::create(name, visu, colModel, ModelLink::Physics(), colChecker);

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createSphere(float radius, Visualization::Color color, const std::string& visualizationType, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }

        VisualizationPtr visu = visualizationFactory->createSphere(radius);
        visu->setColor(color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Sphere(radius));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        if (!visu)
        {
            VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType << endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        //int id = idCounter;
        //idCounter++;

        std::stringstream ss;
        ss << "Sphere";

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker));
        result = Obstacle::create(name, visu, colModel, ModelLink::Physics(), colChecker);

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createCylinder(float radius, float height, Visualization::Color color, const std::string& visualizationType, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }

        VisualizationPtr visu = visualizationFactory->createCylinder(radius, height);
        visu->setColor(color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Cylinder(radius, height));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        if (!visu)
        {
            VR_ERROR << "Could not create cylinder visualization with visu type " << visualizationType << endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        //int id = idCounter;
        //idCounter++;

        std::stringstream ss;
        ss << "Cylinder";

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker));
        result = Obstacle::create(name, visu, colModel, ModelLink::Physics(), colChecker);

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createFromMesh(const TriMeshModelPtr& mesh, const std::string& visualizationType , const CollisionCheckerPtr& colChecker)
    {
        THROW_VR_EXCEPTION_IF(!mesh, "Null data");

        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, nullptr);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }


        VisualizationPtr visu = mesh->getVisualization();

        if (!visu)
        {
            VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType << endl;
            return result;
        }

        //int id = idCounter;
        //idCounter++;

        std::stringstream ss;
        ss << "Mesh";

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker));
        result = Obstacle::create(name, visu, colModel, ModelLink::Physics(), colChecker);

        return result;
    }

    void Obstacle::print(bool printDecoration /*= true*/)
    {
        if (printDecoration)
        {
            cout << "**** Obstacle ****" << endl;
        }

        Model::print();
        //cout << " * id: " << id << endl;

        if (printDecoration)
        {
            cout << endl;
        }
    }

    ObstaclePtr Obstacle::clone(const std::string &name, CollisionCheckerPtr colChecker, float scaling) const
    {
        ReadLockPtr r = getReadLock();
        ModelNodePtr startNode = getRootNode();
        THROW_VR_EXCEPTION_IF(!hasModelNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        ObstaclePtr result(new Obstacle(name));
        _clone(result, startNode, colChecker, true, true, scaling);
        result->filename = filename;

        return result;
    }

    std::string Obstacle::toXML(const std::string& basePath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<Obstacle name='" << name << "'>\n";

        if (getLinks().size()==1)
        {
            ss << getLinks().at(0)->toXML(basePath);
        }

        ss << pre << "</Obstacle>\n";

        return ss.str();
    }


    void Obstacle::setMass(float mass)
    {
        WriteLockPtr l = getWriteLock();
        auto links = getLinks();
        THROW_VR_EXCEPTION_IF(links.size()!=1, "Expecting model with exactly one link...");
        links.at(0)->setMass(mass);
    }

} //  namespace


