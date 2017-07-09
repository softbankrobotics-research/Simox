
#include "Obstacle.h"
#include "../CollisionDetection/CollisionModel.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "Nodes/ModelLink.h"
#include "../Visualization/VisualizationFactory.h"
#include <vector>


namespace VirtualRobot
{

    // obstacle models start with 20000
    int Obstacle::idCounter = 20000;


    Obstacle::Obstacle(const std::string& name, const VisualizationNodePtr& visualization, const CollisionModelPtr& collisionModel, const ModelLink::Physics& p, const CollisionCheckerPtr& colChecker)
        : Model(name, "Obstacle")
    {
        if (name.empty())
        {
            // my id
            id = idCounter++;

            std::stringstream ss;
            ss << "VirtualRobot Obstacle <" << id << ">";
            setName(ss.str());
        }
        else
        {
            if (collisionModel)
            {
                id = collisionModel->getId();
            }
            else
            {
                // my id
                id = idCounter++;
            }
        }

        ModelLinkPtr node(new ModelLink(shared_from_this(),
                                        name,
                                        Eigen::Matrix4f::Identity(),
                                        visualization,
                                        collisionModel,
                                        p,
                                        colChecker ? colChecker : CollisionChecker::getGlobalCollisionChecker()));

        registerModelNode(node);
        setRootNode(node);
        setGlobalPose(Eigen::Matrix4f::Identity());
    }

    Obstacle::~Obstacle()
    {
    }

    int Obstacle::getID() const
    {
        return id;
    }

    VirtualRobot::ObstaclePtr Obstacle::createBox(float width, float height, float depth, VisualizationFactory::Color color, const std::string& visualizationType , const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
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
        VisualizationNodePtr visu = visualizationFactory->createBox(width, height, depth, color.r, color.g, color.b);

        if (!visu)
        {
            VR_ERROR << "Could not create box visualization with visu type " << visualizationType << endl;
            return result;
        }

        //TriMeshModelPtr trimesh = visu->getTriMeshModel();

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Box_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, ModelLink::Physics(), colChecker));
        result->initialize();

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createSphere(float radius, VisualizationFactory::Color color, const std::string& visualizationType, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }

        VisualizationNodePtr visu = visualizationFactory->createSphere(radius, color.r, color.g, color.b);

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

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Sphere_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, ModelLink::Physics(), colChecker));

        result->initialize();

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createCylinder(float radius, float height, VisualizationFactory::Color color, const std::string& visualizationType, const CollisionCheckerPtr& colChecker)
    {
        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = VisualizationFactory::getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }

        VisualizationNodePtr visu = visualizationFactory->createCylinder(radius, height, color.r, color.g, color.b);

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

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Cylinder_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, ModelLink::Physics(), colChecker));

        result->initialize();

        return result;
    }


    VirtualRobot::ObstaclePtr Obstacle::createFromMesh(const TriMeshModelPtr& mesh, const std::string& visualizationType , const CollisionCheckerPtr& colChecker)
    {
        THROW_VR_EXCEPTION_IF(!mesh, "Null data");

        ObstaclePtr result;
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationType.empty())
        {
            visualizationFactory = getGlobalVisualizationFactory();
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationType, NULL);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationType << endl;
            return result;
        }


        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        VisualizationNodePtr visu = visualizationFactory->createTriMeshModelVisualization(mesh, gp);

        if (!visu)
        {
            VR_ERROR << "Could not create sphere visualization with visu type " << visualizationType << endl;
            return result;
        }

        int id = idCounter;
        idCounter++;

        std::stringstream ss;
        ss << "Mesh_" << id;

        std::string name = ss.str();

        CollisionModelPtr colModel(new CollisionModel(visu, name, colChecker, id));
        result.reset(new Obstacle(name, visu, colModel, ModelLink::Physics(), colChecker));

        result->initialize();

        return result;
    }

    void Obstacle::print(bool printDecoration /*= true*/)
    {
        if (printDecoration)
        {
            cout << "**** Obstacle ****" << endl;
        }

        Model::print();
        cout << " * id: " << id << endl;

        if (printDecoration)
        {
            cout << endl;
        }
    }

    Obstacle* Obstacle::_clone(const std::string& name, CollisionCheckerPtr colChecker) const
    {
        VisualizationNodePtr clonedVisualizationNode;

        VR_ASSERT(ModelNode::checkNodeOfType(getRootNode(), ModelNode::ModelNodeType::Link));
        ModelLinkPtr link = std::static_pointer_cast<ModelLink>(getRootNode());

        if (visualizationModel)
        {
            clonedVisualizationNode = link->getVisualization()->clone();
        }

        CollisionModelPtr clonedCollisionModel;

        if (collisionModel)
        {
            clonedCollisionModel = link->getCollisionModel()->clone(colChecker);
        }

        Obstacle* result = new Obstacle(name, clonedVisualizationNode, clonedCollisionModel, physics, colChecker);

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << endl;
            return result;
        }

        result->setGlobalPose(getGlobalPose());

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

        ss << Model::toXML(basePath, tabs);

        ss << pre << "</Obstacle>\n";

        return ss.str();
    }
} //  namespace


