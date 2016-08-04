
#include "RobotFactory.h"
#include "Robot.h"
#include "Nodes/RobotNode.h"
#include "RobotNodeSet.h"
#include "Nodes/RobotNodeRevolute.h"
#include "Nodes/RobotNodePrismatic.h"
#include "Nodes/RobotNodeFixed.h"
#include "Nodes/RobotNodeFixedFactory.h"
#include "EndEffector/EndEffector.h"
#include "Visualization//VisualizationFactory.h"
#include "VirtualRobotException.h"


#include <algorithm>
#include <deque>

namespace VirtualRobot
{


    RobotFactory::RobotFactory()
    {
    }

    RobotFactory::~RobotFactory()
    {
    }


    RobotPtr RobotFactory::createRobot(const std::string& name, const std::string& type)
    {
        RobotPtr result(new LocalRobot(name, type));
        return result;

    }

    bool RobotFactory::initializeRobot(RobotPtr robot,
                                       std::vector<RobotNodePtr >& robotNodes,
                                       std::map< RobotNodePtr, std::vector<std::string> > childrenMap,
                                       RobotNodePtr rootNode
                                      )
    {
        VR_ASSERT(robot);
        bool result = true;

        // check for root
        std::vector<RobotNodePtr >::iterator iter = robotNodes.begin();
        bool foundRoot = false;

        while (iter != robotNodes.end())
        {
            if ((*iter) == rootNode)
            {
                foundRoot = true;
            }

            if ((*iter)->getRobot() != robot)
            {
                THROW_VR_EXCEPTION("Invalid robot node (robot is not set correctly)");
            }

            iter++;
        }

        THROW_VR_EXCEPTION_IF(!foundRoot, "Invalid robot node (root is not available)");

        // process children
        std::map< RobotNodePtr, std::vector< std::string > >::iterator iterC = childrenMap.begin();

        while (iterC != childrenMap.end())
        {
            std::vector< std::string > childNames = iterC->second;
            RobotNodePtr node = iterC->first;

            for (size_t i = 0; i < childNames.size(); i++)
            {
                std::string childName = childNames[i];

                if (!robot->hasRobotNode(childName))
                {
                    THROW_VR_EXCEPTION("Robot " << robot->getName() << ": corrupted RobotNode <" << node->getName() << " child :" << childName << " does not exist...");
                }

                RobotNodePtr c = robot->getRobotNode(childName);
                node->attachChild(c);
            }

            iterC++;
        }

        // register root (performs an initialization of all robot nodes)
        robot->setRootNode(rootNode);

        for (size_t i = 0; i < robotNodes.size(); i++)
        {
            if (!robotNodes[i]->getParent() && robotNodes[i] != rootNode)
            {
                VR_ERROR << "RobotNode " << robotNodes[i]->getName() << " is not connected to kinematic structure..." << endl;
            }
        }

        return result;
    }



    struct robotNodeDef
    {
        std::string name;
        std::vector<std::string> children;
    };

    struct robotStructureDef
    {
        std::string rootName;
        std::vector<robotNodeDef> parentChildMapping;
    };

    RobotPtr RobotFactory::cloneInversed(RobotPtr robot, const std::string& newRootName, bool cloneRNS, bool cloneEEF)
    {
        VR_ASSERT(robot);

        RobotNodePtr newRoot = robot->getRobotNode(newRootName);

        if (!newRoot)
        {
            VR_ERROR << "No node " << newRootName << endl;
        }

        RobotFactory::robotStructureDef newStructure;
        newStructure.rootName = newRootName;

        typedef std::pair<RobotNodePtr, RobotNodePtr> RobotTreeEdge;

        std::deque<RobotTreeEdge> edges;
        RobotTreeEdge rootEdge;
        rootEdge.second = newRoot;
        edges.push_back(rootEdge);

        while (!edges.empty())
        {
            RobotTreeEdge currentEdge = edges.front();
            edges.pop_front();

            RobotNodePtr parent = boost::dynamic_pointer_cast<RobotNode>(currentEdge.second->getParent());

            std::vector<SceneObjectPtr> children = currentEdge.second->getChildren();
            RobotFactory::robotNodeDef rnDef;
            rnDef.name = currentEdge.second->getName();

            // invert transformation of old parent
            if (parent && parent != currentEdge.first)
            {
                rnDef.invertTransformation.push_back(true);
                rnDef.children.push_back(parent->getName());

                RobotTreeEdge edge;
                edge.first = currentEdge.second;
                edge.second = parent;

                BOOST_ASSERT(edge.second);
                edges.push_back(edge);
            }

            for (unsigned i = 0; i < children.size(); i++)
            {
                if (children[i] != currentEdge.first)
                {
                    RobotNodePtr childNode = boost::dynamic_pointer_cast<RobotNode>(children[i]);

                    // not a robot node
                    if (!childNode)
                    {
                        continue;
                    }

                    rnDef.children.push_back(children[i]->getName());
                    rnDef.invertTransformation.push_back(false);
                    RobotTreeEdge edge;
                    edge.second = childNode;
                    edge.first = currentEdge.second;

                    BOOST_ASSERT(edge.second);
                    edges.push_back(edge);
                }
            }

            newStructure.parentChildMapping.push_back(rnDef);
        }


        RobotPtr r = RobotFactory::cloneChangeStructure(robot, newStructure);

        if (cloneRNS)
        {
            std::vector<VirtualRobot::RobotNodeSetPtr> robotNodeSets;
            for (RobotNodeSetPtr rns : robot->getRobotNodeSets())
            {
                robotNodeSets.push_back(robot->getRobotNodeSet(rns->getName())->clone(r));
            }
        }

        if (cloneEEF)
        {
            // Copy end effectors
            for(auto &eef : robot->getEndEffectors())
            {
                eef->clone(r);
            }
        }

        return r;
    }


    RobotPtr RobotFactory::cloneChangeStructure(RobotPtr robot, const std::string& startNode, const std::string& endNode)
    {
        VR_ASSERT(robot);

        if (!robot->hasRobotNode(startNode))
        {
            VR_ERROR << "No node with name " << startNode << endl;
            return RobotPtr();
        }

        if (!robot->hasRobotNode(endNode))
        {
            VR_ERROR << "No node with name " << endNode << endl;
            return RobotPtr();
        }

        if (!robot->getRobotNode(startNode)->hasChild(endNode, true))
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << endl;
            return RobotPtr();
        }

        std::vector<std::string> nodes;
        std::string currentNodeName = endNode;
        RobotNodePtr rn = robot->getRobotNode(currentNodeName);

        while (rn && !(rn->getName() == startNode))
        {
            currentNodeName = rn->getName();
            nodes.push_back(currentNodeName);
            rn = boost::dynamic_pointer_cast<RobotNode>(rn->getParent());
        }

        if (!rn)
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << endl;
            return RobotPtr();
        }

        nodes.push_back(startNode);
        //std::reverse(nodes.begin(), nodes.end());

        RobotFactory::robotStructureDef newStructure;
        newStructure.rootName = endNode;

        for (size_t i = 0; i < nodes.size() - 1; i++)
        {
            RobotFactory::robotNodeDef rnDef;
            rnDef.name = nodes[i];
            rnDef.children.push_back(nodes[i + 1]);
            rnDef.invertTransformation.push_back(true);
            newStructure.parentChildMapping.push_back(rnDef);
        }

        RobotFactory::robotNodeDef rnDef;
        rnDef.name = nodes[nodes.size() - 1];
        rnDef.invertTransformation.push_back(true);
        newStructure.parentChildMapping.push_back(rnDef);


        return RobotFactory::cloneChangeStructure(robot, newStructure);
    }

    bool RobotFactory::attach(RobotPtr robot, SceneObjectPtr o, RobotNodePtr rn, const Eigen::Matrix4f& transformation)
    {
        if (!robot || !o || !rn)
        {
            return false;
        }

        std::string name = o->getName();

        if (robot->hasRobotNode(name))
        {
            VR_WARNING << "RN with name " << name << " already present" << endl;
            return false;
        }

        SceneObject::Physics p = o->physics;
        VisualizationNodePtr v;

        if (o->getVisualization())
        {
            v = o->getVisualization()->clone();
        }

        CollisionModelPtr c;

        if (o->getCollisionModel())
        {
            c = o->getCollisionModel()->clone();
        }

        auto rnf = RobotNodeFixedFactory::createInstance(NULL);
        RobotNodePtr newRN = rnf->createRobotNode(robot, name, v, c, 0, 0, 0, transformation, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);
        rn->attachChild(newRN);
        newRN->initialize(rn);
        return true;
    }

    bool RobotFactory::detach(RobotPtr robot, RobotNodePtr rn)
    {
        if (!robot || !rn || !rn->getParent())
        {
            return false;
        }

        if (!robot->hasRobotNode(rn))
        {
            return false;
        }

        rn->getParent()->detachChild(rn);
        robot->deregisterRobotNode(rn);
        return true;
    }

    RobotPtr RobotFactory::cloneChangeStructure(RobotPtr robot, robotStructureDef& newStructure)
    {
        VR_ASSERT(robot);

        if (!robot->hasRobotNode(newStructure.rootName))
        {
            VR_ERROR << "No root with name " << newStructure.rootName << endl;
            return RobotPtr();
        }

        std::map<std::string, RobotNodePtr> newNodes;
        RobotPtr newRobot = createRobot(robot->getName(), robot->getType() + "_restructured_" + newStructure.rootName);
        RobotNodePtr rn = robot->getRobotNode(newStructure.rootName);
        rn = rn->clone(newRobot, false);
        newNodes[newStructure.rootName] = rn;
        newRobot->setRootNode(newNodes[newStructure.rootName]);

        std::string nodeName;
        typedef std::map < RobotNodePtr,
                Eigen::Matrix4f,
                std::less<RobotNodePtr>,
                Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4f> > >
                NodeTransformationMapT;

        NodeTransformationMapT localTransformations;
        std::map<RobotNodePtr, VisualizationNodePtr> visuMap;
        std::map<RobotNodePtr, CollisionModelPtr> colMap;
        std::map<RobotNodePtr, SceneObject::Physics> physicsMap;
        std::map<RobotNodePtr, std::vector<SensorPtr> > sensorMap;
        std::map<RobotNodePtr, bool> directionInversion;

        for (size_t i = 0; i < newStructure.parentChildMapping.size(); i++)
        {
            if (!robot->hasRobotNode(newStructure.parentChildMapping[i].name))
            {
                VR_ERROR << "Error in parentChildMapping, no node with name " << newStructure.parentChildMapping[i].name << endl;
                return RobotPtr();
            }

            nodeName = newStructure.parentChildMapping[i].name;

            if (newNodes.find(nodeName) == newNodes.end())
            {
                rn = robot->getRobotNode(nodeName);
                rn = rn->clone(newRobot, false);
                newNodes[nodeName] = rn;
            }

            RobotNodePtr parent = newNodes[newStructure.parentChildMapping[i].name];

            for (size_t j = 0; j < newStructure.parentChildMapping[i].children.size(); j++)
            {
                nodeName = newStructure.parentChildMapping[i].children[j];

                if (!robot->hasRobotNode(nodeName))
                {
                    VR_ERROR << "Error in parentChildMapping, no child node with name " << nodeName << endl;
                    return RobotPtr();
                }

                if (newNodes.find(nodeName) == newNodes.end())
                {
                    rn = robot->getRobotNode(nodeName);
                    rn = rn->clone(newRobot, false);
                    newNodes[nodeName] = rn;
                }

                //children.push_back(newNodes[nodeName]);
                RobotNodePtr child = newNodes[nodeName];
                parent->attachChild(child);

                if (newStructure.parentChildMapping[i].invertTransformation[j])
                {
                    Eigen::Matrix4f tr = parent->getLocalTransformation().inverse();
                    localTransformations[child] = tr;
                    // we also need to invert the direction
                    directionInversion[child] = true;

                    // check for models
                    if (child->getVisualization())
                    {
                        VisualizationNodePtr v = child->getVisualization();
                        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
                        Eigen::Matrix4f tr2 = tr;
                        //tr2.block(0, 3, 3, 1) *= 0.001f; // m is needed here?
                        vf->applyDisplacement(v, tr2);
                        visuMap[parent] = v;

                        for (size_t pr = 0; pr < v->primitives.size(); pr++)
                        {
                            v->primitives[pr]->transform = tr * v->primitives[pr]->transform;
                        }
                    }

                    if (child->getCollisionModel())
                    {
                        CollisionModelPtr c = child->getCollisionModel();
                        VisualizationNodePtr v = child->getCollisionModel()->getVisualization();
                        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
                        Eigen::Matrix4f tr2 = tr;
                        //tr2.block(0, 3, 3, 1) *= 0.001f; // m is needed here?
                        vf->applyDisplacement(v, tr2);
                        v->createTriMeshModel(); // update trimesh model
                        c->setVisualization(v);
                        colMap[parent] = c;

                        for (size_t pr = 0; pr < v->primitives.size(); pr++)
                        {
                            v->primitives[pr]->transform = tr * v->primitives[pr]->transform;
                        }

                    }

                    // exchange physics
                    physicsMap[parent] = child->physics;
                    // change local com to new coord system
                    Eigen::Vector4f l;
                    SceneObject::Physics p = physicsMap[parent];
                    Eigen::Vector3f loc = p.localCoM;
                    l << loc(0), loc(1), loc(2), 1.0f;
                    physicsMap[parent].localCoM = (tr * l).head(3);

                    if (physicsMap.find(child) == physicsMap.end())
                    {
                        // no entry for child, set it to empty, may be overwritten later on
                        physicsMap[child] = SceneObject::Physics();
                    }

                    // exchange sensors
                    std::vector<SceneObjectPtr> childChildren = robot->getRobotNode(nodeName)->getChildren();

                    for (auto cc : childChildren)
                    {
                        SensorPtr cs = boost::dynamic_pointer_cast<Sensor>(cc);

                        if (cs)
                        {
                            // cloning sensor
                            SensorPtr newSensor = cs->clone(parent);
                            sensorMap[parent].push_back(newSensor);
                        }

                    }

                }
                else
                {
                    localTransformations[child] = child->getLocalTransformation();
                    directionInversion[child] = false;

                    if (child->getVisualization())
                    {
                        visuMap[child] = child->getVisualization();
                    }

                    if (child->getCollisionModel())
                    {
                        colMap[child] = child->getCollisionModel();
                    }

                    // clone sensors
                    std::vector<SceneObjectPtr> childChildren = robot->getRobotNode(nodeName)->getChildren();

                    for (auto cc : childChildren)
                    {
                        SensorPtr cs = boost::dynamic_pointer_cast<Sensor>(cc);

                        if (cs)
                        {
                            // cloning sensor
                            SensorPtr newSensor = cs->clone(child);
                            sensorMap[child].push_back(newSensor);
                        }

                    }
                }
            }

            // if parent has no parent: reset local transformation
            if (localTransformations.find(parent) == localTransformations.end())
            {
                localTransformations[parent] = Eigen::Matrix4f::Identity();
                directionInversion[parent] = false;
            }
        }

        // apply all transformations
        NodeTransformationMapT::iterator it = localTransformations.begin();

        while (it != localTransformations.end())
        {
            it->first->localTransformation = it->second;
            std::map<RobotNodePtr, bool>::iterator inv_it = directionInversion.find(it->first);
            VR_ASSERT(inv_it != directionInversion.end());
            if (inv_it->second)
            {
                RobotNodeRevolutePtr rotJoint = boost::dynamic_pointer_cast<RobotNodeRevolute>(it->first);
                if (rotJoint)
                {
                    rotJoint->jointRotationAxis *= -1.0f;
                }

                RobotNodePrismaticPtr prismaticJoint = boost::dynamic_pointer_cast<RobotNodePrismatic>(it->first);
                if (prismaticJoint)
                {
                    prismaticJoint->jointTranslationDirection *= -1.0f;
                }
            }

            it++;
        }

        std::vector<RobotNodePtr> nodes = newRobot->getRobotNodes();

        for (size_t i = 0; i < nodes.size(); i++)
        {
            if (visuMap.find(nodes[i]) != visuMap.end())
            {
                nodes[i]->setVisualization(visuMap[nodes[i]]);
            }
            else
            {
                nodes[i]->setVisualization(VisualizationNodePtr());
            }

            if (colMap.find(nodes[i]) != colMap.end())
            {
                nodes[i]->setCollisionModel(colMap[nodes[i]]);
            }
            else
            {
                nodes[i]->setCollisionModel(CollisionModelPtr());
            }

            if (physicsMap.find(nodes[i]) != physicsMap.end())
            {
                nodes[i]->physics = physicsMap[nodes[i]];
            }

            if (sensorMap.find(nodes[i]) != sensorMap.end())
            {
                auto sensors = sensorMap[nodes[i]];

                for (auto s : sensors)
                {
                    nodes[i]->registerSensor(s);
                }
            }
        }

        newRobot->getRootNode()->initialize();

        return newRobot;
    }


    RobotPtr RobotFactory::clone(RobotPtr robot, const std::string& name, CollisionCheckerPtr collisionChecker, float scaling)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        if (!collisionChecker)
        {
            collisionChecker = robot->getCollisionChecker();
        }

        VirtualRobot::RobotPtr result = robot->extractSubPart(robot->getRootNode(), robot->getType(), name, true, true, collisionChecker, scaling);
        result->setGlobalPose(robot->getGlobalPose());
        return result;
    }

    void RobotFactory::getChildNodes(RobotNodePtr nodeA, RobotNodePtr nodeExclude, std::vector<RobotNodePtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector < SceneObjectPtr > children = nodeA->getChildren();
        std::vector<RobotNodePtr> childNodes;

        for (size_t i = 0; i < children.size(); i++)
        {
            SceneObjectPtr c = children[i];
            RobotNodePtr cRN = boost::dynamic_pointer_cast<RobotNode>(c);

            if (cRN && cRN != nodeExclude)
            {
                appendNodes.push_back(cRN);
                getChildNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }

    void RobotFactory::getChildSensorNodes(RobotNodePtr nodeA, RobotNodePtr nodeExclude, std::vector<SensorPtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector < SceneObjectPtr > children = nodeA->getChildren();
        std::vector<RobotNodePtr> childNodes;

        for (size_t i = 0; i < children.size(); i++)
        {
            SceneObjectPtr c = children[i];
            SensorPtr cS = boost::dynamic_pointer_cast<Sensor>(c);
            RobotNodePtr cRN = boost::dynamic_pointer_cast<RobotNode>(c);

            if (cS)
            {
                appendNodes.push_back(cS);
            }

            if (cRN && cRN != nodeExclude)
            {
                getChildSensorNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }

    RobotNodePtr RobotFactory::accumulateTransformations(RobotPtr robot, RobotNodePtr nodeA, RobotNodePtr nodeAClone, RobotNodePtr nodeB, Eigen::Matrix4f& storeTrafo)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        auto rnf = RobotNodeFixedFactory::createInstance(NULL);
        SceneObject::Physics p;
        VisualizationNodePtr v;
        CollisionModelPtr c;

        if (nodeA == nodeB)
        {
            RobotNodePtr newRN = rnf->createRobotNode(robot, nodeA->getName() + "_non_trafo", v, c, 0, 0, 0, Eigen::Matrix4f::Identity(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);
            return newRN;
        }

        std::vector<RobotNodePtr> childNodes;
        std::vector<SensorPtr> childSensorNodes;

        getChildNodes(nodeA, nodeB, childNodes);
        getChildSensorNodes(nodeA, nodeB, childSensorNodes);

        if (childNodes.size() == 0)
        {
            return RobotNodePtr();
        }

        storeTrafo = Eigen::Matrix4f::Identity();

        if (nodeA && nodeB)
        {
            Eigen::Matrix4f startPose = nodeA->getGlobalPose();
            Eigen::Matrix4f goalPose = nodeB->getParent()->getGlobalPose();
            storeTrafo = goalPose * startPose.inverse();
        }

        RobotNodePtr res = createUnitedRobotNode(robot, childNodes, nodeA, nodeAClone, Eigen::Matrix4f::Identity(), childSensorNodes);

        return res;
    }

    RobotNodePtr RobotFactory::createUnitedRobotNode(RobotPtr robot, const std::vector< RobotNodePtr >& nodes, RobotNodePtr parent, RobotNodePtr parentClone, const Eigen::Matrix4f& trafo, const std::vector<SensorPtr> &sensors)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");

        auto rnf = RobotNodeFixedFactory::createInstance(NULL);
        SceneObject::Physics p;
        VisualizationNodePtr v;
        CollisionModelPtr c;
        std::string name = "root";

        if (parentClone)
        {
            name = parentClone->getName() + "_FixedTrafo";
        }

        if (nodes.size() == 0)
        {
            RobotNodePtr newRN = rnf->createRobotNode(robot, name, v, c, 0, 0, 0, trafo, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);

            if (parentClone)
            {
                newRN->initialize(parentClone);
            }

            // attach sensors
            for (size_t i = 0; i < sensors.size(); i++)
            {
                SensorPtr s = sensors[i]->clone(newRN);
            }

            return newRN;
        }

        VisualizationFactoryPtr vf = VisualizationFactory::first(NULL);
        std::vector < VisualizationNodePtr > visus;
        std::vector < VisualizationNodePtr > colVisus;
        float kg = 0;


        for (size_t i = 0; i < nodes.size(); i++)
        {
            if (nodes[i]->getVisualization())
            {
                visus.push_back(nodes[i]->getVisualization());
            }

            if (nodes[i]->getCollisionModel() && nodes[i]->getCollisionModel()->getVisualization())
            {
                colVisus.push_back(nodes[i]->getCollisionModel()->getVisualization());
            }

            kg += nodes[i]->getMass();
        }

        if (visus.size() > 0)
        {
            v = vf->createUnitedVisualization(visus)->clone();
            if (parent)
            {
                Eigen::Matrix4f invTr = parent->getGlobalPose().inverse();
                vf->applyDisplacement(v, invTr);
            }
        }

        if (colVisus.size() > 0)
        {
            VisualizationNodePtr colVisu = vf->createUnitedVisualization(colVisus)->clone();
            if (parent)
            {
                Eigen::Matrix4f invTr = parent->getGlobalPose().inverse();
                vf->applyDisplacement(colVisu, invTr);
            }
            c.reset(new CollisionModel(colVisu, nodes[0]->getName()));
        }

        p.massKg = kg;

        RobotNodePtr newRN = rnf->createRobotNode(robot, name, v, c, 0, 0, 0, trafo, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);

        newRN->initialize(parentClone);

        // attach sensors
        for (size_t i = 0; i < sensors.size(); i++)
        {
            SensorPtr s = sensors[i]->clone(newRN);
            Eigen::Matrix4f trafoToNewRN = parent?parent->getGlobalPose() * trafo:trafo;
            Eigen::Matrix4f t = trafoToNewRN.inverse() * sensors[i]->getGlobalPose();
            s->setRobotNodeToSensorTransformation(t);
        }

        return newRN;
    }


    RobotPtr RobotFactory::cloneSubSet(RobotPtr robot, RobotNodeSetPtr rns, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        THROW_VR_EXCEPTION_IF(!rns, "NULL data");
        THROW_VR_EXCEPTION_IF(rns->getRobot() != robot, "Inconsitent data");

        std::vector< RobotNodePtr > nodes = rns->getAllRobotNodes();
        THROW_VR_EXCEPTION_IF(nodes.size() == 0, "0 data");

        // ensure tcp is part of nodes
        //if (rns->getTCP() && !rns->hasRobotNode(rns->getTCP()))
        //    nodes.push_back(rns->getTCP());

        // ensure kinemtic root is part of nodes
        if (rns->getKinematicRoot() && !rns->hasRobotNode(rns->getKinematicRoot()))
            nodes.insert(nodes.begin(),rns->getKinematicRoot());

        RobotNodePtr startNode = rns->getKinematicRoot();
        if (!startNode)
            startNode = nodes[0];

        for (size_t i = 1; i < nodes.size(); i++)
        {
            RobotNodePtr a = nodes[i - 1];
            RobotNodePtr b = nodes[i];

            if (!a->hasChild(b, true))
            {
                std::stringstream ss;
                ss << "Node " << a->getName() << " is not parent of " << b->getName();
                THROW_VR_EXCEPTION(ss.str());
            }
        }

        // first create initial node
        std::vector< RobotNodePtr > initialNodes = startNode->getAllParents();
        // check for static nodes which are not parent of startNode
        std::vector< RobotNodePtr > allNodes = robot->getRobotNodes();
        for (size_t i=0;i<allNodes.size();i++)
        {
            RobotNodePtr rn = allNodes[i];
            bool isFixed = true;
            for (size_t j = 0; j < nodes.size(); j++)
            {
                if (rn->hasChild(nodes[j],true))
                {
                    isFixed = false;
                    break;
                }
            }
            if (isFixed && std::find(initialNodes.begin(), initialNodes.end(),rn) == initialNodes.end())
            {
                // check if rn is child of the nodes in the rns
                if (!startNode->hasChild(rn, true))
                    initialNodes.push_back(rn);
            }
        }

        RobotPtr result(new LocalRobot(name, robot->getType()));

        Eigen::Matrix4f currentTrafo = Eigen::Matrix4f::Identity();

        if (startNode->getParent())
        {
            currentTrafo = startNode->getParent()->getGlobalPose();
        }

        // collect sensor nodes
        std::vector<SensorPtr> childSensorNodes;
        for (size_t i = 0; i < initialNodes.size(); i++)
        {
            RobotNodePtr rn = initialNodes[i];
            std::vector<SceneObjectPtr> c = rn->getChildren();
            for (size_t j = 0; j < c.size(); j++)
            {
                SensorPtr s = boost::dynamic_pointer_cast<Sensor>(c[j]);
                if (s)
                    childSensorNodes.push_back(s);
            }
        }

        RobotNodePtr rootNode = createUnitedRobotNode(result, initialNodes, RobotNodePtr(), RobotNodePtr(), Eigen::Matrix4f::Identity(), childSensorNodes);
        result->setRootNode(rootNode);
        RobotNodePtr currentParent = rootNode;
    
        for (size_t i = 0; i < nodes.size(); i++)
        {
            RobotNodePtr newNode = nodes[i]->clone(result, false, currentParent);
            Eigen::Matrix4f newTrafo = currentTrafo * newNode->getLocalTransformation();
            newNode->setLocalTransformation(newTrafo);
            currentParent = newNode;
            currentTrafo.setIdentity();

            RobotNodePtr secondNode;

            if (i < nodes.size() - 1)
            {
                secondNode = nodes[i + 1];
            } else
            {
                cout << "end";
            }

            RobotNodePtr newNodeFixed = accumulateTransformations(result, nodes[i], newNode, secondNode, currentTrafo);

            if (newNodeFixed)
            {
                currentParent = newNodeFixed;
            }
        }

        result->setGlobalPose(robot->getGlobalPose());
        return result;
    }

    RobotPtr RobotFactory::cloneUniteSubsets(RobotPtr robot, const std::string& name, std::vector<std::string> uniteWithAllChildren)
    {
        THROW_VR_EXCEPTION_IF(!robot, "NULL data");
        if (uniteWithAllChildren.size() == 0)
            return RobotFactory::clone(robot, robot->getName());

        for (size_t i = 0; i < uniteWithAllChildren.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!robot->hasRobotNode(uniteWithAllChildren[i]), "Could not find RobotNode in robot");
        }


        RobotPtr result(new LocalRobot(name, robot->getType()));


        RobotNodePtr currentNode = robot->getRootNode();
        RobotNodePtr currentNodeClone = currentNode->clone(result, false);
        result->setRootNode(currentNodeClone);

        cloneRecursiveUnite(result, currentNode, currentNodeClone, uniteWithAllChildren);

        // clone RNS
        std::vector<RobotNodeSetPtr> rnsets = robot->getRobotNodeSets();
        for (size_t i = 0; i < rnsets.size(); i++)
        {
            RobotNodeSetPtr rns = rnsets[i];
            
            bool ok = true;
            for (size_t j = 0; j < uniteWithAllChildren.size(); j++)
            {
                RobotNodePtr rn = robot->getRobotNode(uniteWithAllChildren[j]);
                std::vector<RobotNodePtr> allChildren;
                rn->collectAllRobotNodes(allChildren);
                for (size_t k = 0; k < allChildren.size(); k++)
                {
                    if (allChildren[k] == rn)
                        continue;
                    if (rns->hasRobotNode(allChildren[k]->getName()))
                    {
                        ok = false;
                        break;
                    }
                }
            }
            if (ok)
            {
                rns->clone(result);
            }
        }

        result->setGlobalPose(robot->getGlobalPose());
        return result;
    }

    void RobotFactory::cloneRecursiveUnite(RobotPtr robot, RobotNodePtr currentNode, RobotNodePtr currentNodeClone, std::vector<std::string> uniteWithAllChildren)
    {
        std::vector<SceneObjectPtr> c = currentNode->getChildren();


        for (size_t i = 0; i < c.size(); i++)
        {
            if (std::find(uniteWithAllChildren.begin(), uniteWithAllChildren.end(), c[i]->getName()) != uniteWithAllChildren.end())
            {
                RobotNodePtr currentRN = boost::dynamic_pointer_cast<RobotNode>(c[i]);
                THROW_VR_EXCEPTION_IF(!currentRN, "Only RN allowed in list");
                RobotNodePtr currentRNClone = currentRN->clone(robot, false, currentNodeClone);

                std::vector<RobotNodePtr> childNodes;
                std::vector<SensorPtr> childSensorNodes;

                getChildNodes(currentRN, RobotNodePtr(), childNodes);
                getChildSensorNodes(currentRN, RobotNodePtr(), childSensorNodes);

                if (childNodes.size() > 0 || childSensorNodes.size() > 0)
                {
                    RobotNodePtr res = createUnitedRobotNode(robot, childNodes, currentRN, currentRNClone, Eigen::Matrix4f::Identity(), childSensorNodes);
                    // res is automatically added
                }
            }
            else
            {
                RobotNodePtr currentRN = boost::dynamic_pointer_cast<RobotNode>(c[i]);
                if (currentRN)
                {
                    RobotNodePtr currentRNClone = currentRN->clone(robot, false, currentNodeClone);
                    cloneRecursiveUnite(robot, currentRN, currentRNClone, uniteWithAllChildren);
                }
                else
                {
                    SensorPtr s = boost::dynamic_pointer_cast<Sensor>(c[i]);
                    if (s)
                    {
                        s->clone(currentNodeClone);
                    } else
                        VR_INFO << "Skipping node " << c[i]->getName() << endl;
                }
            }
 
        }

    }

} // namespace VirtualRobot
