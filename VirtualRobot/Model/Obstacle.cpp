
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


    Obstacle::Obstacle(const std::string& name, const std::string &type)
        : Model(name, type)
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

        m->registerNode(node);
        m->setRootNode(node);
        m->setGlobalPose(Eigen::Matrix4f::Identity());

        return m;
    }

    VirtualRobot::ObstaclePtr Obstacle::createBox(float width, float height, float depth, Visualization::Color color, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::getInstance();

        /*
        std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Box(width,height,depth));
        primitives.push_back(p);

        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */
        VisualizationPtr visu = visualizationFactory->createBox(width, height, depth);
        VR_ASSERT(visu);
        visu->setColor(color);

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


    VirtualRobot::ObstaclePtr Obstacle::createSphere(float radius, Visualization::Color color, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::getInstance();

        VisualizationPtr visu = visualizationFactory->createSphere(radius);
        VR_ASSERT(visu);
        visu->setColor(color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Sphere(radius));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */

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


    VirtualRobot::ObstaclePtr Obstacle::createCylinder(float radius, float height, Visualization::Color color, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::getInstance();

        VisualizationPtr visu = visualizationFactory->createCylinder(radius, height);
        VR_ASSERT(visu);
        visu->setColor(color);

        /*std::vector<Primitive::PrimitivePtr> primitives;
        Primitive::PrimitivePtr p(new Primitive::Cylinder(radius, height));
        primitives.push_back(p);
        VisualizationNodePtr visu = visualizationFactory->getVisualizationFromPrimitives(primitives,false,color);
        */

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


    VirtualRobot::ObstaclePtr Obstacle::createFromMesh(const TriMeshModelPtr& mesh, const CollisionCheckerPtr& colChecker)
    {
        THROW_VR_EXCEPTION_IF(!mesh, "Null data");

        ObstaclePtr result;
        VisualizationPtr visu = mesh->getVisualization();
        VR_ASSERT(visu);

        //int id = idCounter;
        //idCounter++;

        std::stringstream ss;
        ss << "Mesh";

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker));
        result = Obstacle::create(name, visu, colModel, ModelLink::Physics(), colChecker);

        return result;
    }

    ObstaclePtr Obstacle::clone(const std::string &name, CollisionCheckerPtr colChecker, float scaling) const
    {
        ReadLockPtr r = getReadLock();
        ModelNodePtr startNode = getRootNode();
        THROW_VR_EXCEPTION_IF(!hasNode(startNode), " StartJoint is not part of this robot");
        THROW_VR_EXCEPTION_IF(scaling <= 0, " Scaling must be >0.");

        ObstaclePtr result(new Obstacle(name));
        _clone(result, startNode, colChecker, true, true, scaling);
        result->filename = filename;

        return result;
    }

    std::string Obstacle::toXML(const std::string& basePath, const std::string &modelPath, bool storeEEF, bool storeRNS, bool storeAttachments, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<Obstacle name='" << getName() << "'>\n";

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

    CollisionModelPtr Obstacle::getCollisionModel() const
    {
        const auto ms = getCollisionModels();
        VR_ASSERT(ms.size() == 1);
        return ms[0];
    }

    ModelLink::Physics::SimulationType Obstacle::getSimulationType() const
    {
        const auto l = getLinks();
        VR_ASSERT(l.size() == 1);
        return l[0]->getSimulationType();
    }

    void Obstacle::setSimulationType(ModelLink::Physics::SimulationType simType)
    {
        const auto l = getLinks();
        VR_ASSERT(l.size() == 1);
        l[0]->setSimulationType(simType);
    }

} //  namespace


