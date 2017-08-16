
#include "ModelFactory.h"
#include "Model.h"
#include "Nodes/ModelNode.h"
#include "ModelNodeSet.h"
#include "Nodes/ModelJointRevolute.h"
#include "Nodes/ModelJointPrismatic.h"
#include "Nodes/ModelJointFixed.h"
#include "../EndEffector/EndEffector.h"
#include "../Visualization//VisualizationFactory.h"
#include "../VirtualRobotException.h"


#include <algorithm>
#include <deque>

namespace VirtualRobot
{


    ModelFactory::ModelFactory()
    {
    }

    ModelFactory::~ModelFactory()
    {
    }


    ModelPtr ModelFactory::createModel(const std::string& name, const std::string& type)
    {
        ModelPtr result(new Model(name, type));
        return result;

    }

    bool ModelFactory::initializeModel(const ModelPtr& model,
                                       std::vector<ModelNodePtr >& modelNodes,
                                       const std::map< ModelNodePtr, std::vector<std::string> >& childrenMap,
                                       ModelNodePtr& rootNode
                                      )
    {
        VR_ASSERT(model);
        bool result = true;

        // check for root
        std::vector<ModelNodePtr >::iterator iter = modelNodes.begin();
        bool foundRoot = false;

        while (iter != modelNodes.end())
        {
            if ((*iter) == rootNode)
            {
                foundRoot = true;
            }

            if ((*iter)->getModel() != model)
            {
                THROW_VR_EXCEPTION("Invalid model node (model is not set correctly)");
            }

            iter++;
        }

        THROW_VR_EXCEPTION_IF(!foundRoot, "Invalid model node (root is not available)");


        // register root
        if (!model->hasModelNode(rootNode->getName()))
            model->registerModelNode(rootNode);
        model->setRootNode(rootNode, false);

        // go through tree and attach nodes according to parent-child mapping
        std::vector<ModelNodePtr> openNodes;
        ModelNodePtr& currentNode = rootNode;
        while (currentNode)
        {
            if (childrenMap.find(currentNode) != childrenMap.end())
            {
                // children in map, attach them to parent
                const std::vector<std::string> &currentChildren = childrenMap.at(currentNode);
                for (auto childName : currentChildren)
                {
                    ModelNodePtr c;
                    for (auto m : modelNodes)
                        if (m->getName() == childName)
                        {
                            c = m;
                            break;
                        }
                    THROW_VR_EXCEPTION_IF(!c, "Corrupt children map, could not find node with name " + childName);
                    if (!model->hasModelNode(c))
                    {
                        model->registerModelNode(c);
                    }
                    currentNode->attachChild(c, false);
                    openNodes.push_back(c);
                }
            }
            if (openNodes.size() > 0)
            {
                currentNode = openNodes.at(openNodes.size() - 1);
                openNodes.pop_back();
            }
            else
            {
                currentNode.reset();
                break;
            }
        }
        THROW_VR_EXCEPTION_IF(openNodes.size() != 0, "Internal error");
        THROW_VR_EXCEPTION_IF(model->getModelNodes().size() != modelNodes.size(), "Could not attach all model nodes to model");

        for (size_t i = 0; i < modelNodes.size(); i++)
        {
            if (!modelNodes[i]->getParentNode() && modelNodes[i] != rootNode)
            {
                VR_ERROR << "ModelNode " << modelNodes[i]->getName() << " is not connected to kinematic structure..." << endl;
            }
        }

        model->getRootNode()->updatePose();

        return result;
    }



    struct modelNodeDef
    {
        std::string name;
        std::vector<std::string> children;
    };

    struct modelStructureDef
    {
        std::string rootName;
        std::vector<modelNodeDef> parentChildMapping;
    };
    /*
    ModelPtr ModelFactory::cloneInversed(const ModelPtr& model, const std::string& newRootName, bool cloneRNS, bool cloneEEF)
    {
        VR_ASSERT(model);

        ModelNodePtr newRoot = model->getModelNode(newRootName);

        if (!newRoot)
        {
            VR_ERROR << "No node " << newRootName << endl;
        }

        ModelFactory::modelStructureDef newStructure;
        newStructure.rootName = newRootName;

        typedef std::pair<ModelNodePtr, ModelNodePtr> ModelTreeEdge;

        std::deque<ModelTreeEdge> edges;
        ModelTreeEdge rootEdge;
        rootEdge.second = newRoot;
        edges.push_back(rootEdge);

        while (!edges.empty())
        {
            ModelTreeEdge currentEdge = edges.front();
            edges.pop_front();

            ModelNodePtr parent = std::dynamic_pointer_cast<ModelNode>(currentEdge.second->getParent());

            std::vector<SceneObjectPtr> children = currentEdge.second->getChildren();
            ModelFactory::modelNodeDef rnDef;
            rnDef.name = currentEdge.second->getName();

            // invert transformation of old parent
            if (parent && parent != currentEdge.first)
            {
                rnDef.invertTransformation.push_back(true);
                rnDef.children.push_back(parent->getName());

                ModelTreeEdge edge;
                edge.first = currentEdge.second;
                edge.second = parent;

                BOOST_ASSERT(edge.second);
                edges.push_back(edge);
            }

            for (unsigned i = 0; i < children.size(); i++)
            {
                if (children[i] != currentEdge.first)
                {
                    ModelNodePtr childNode = std::dynamic_pointer_cast<ModelNode>(children[i]);

                    // not a model node
                    if (!childNode)
                    {
                        continue;
                    }

                    rnDef.children.push_back(children[i]->getName());
                    rnDef.invertTransformation.push_back(false);
                    ModelTreeEdge edge;
                    edge.second = childNode;
                    edge.first = currentEdge.second;

                    BOOST_ASSERT(edge.second);
                    edges.push_back(edge);
                }
            }

            newStructure.parentChildMapping.push_back(rnDef);
        }


        ModelPtr r = ModelFactory::cloneChangeStructure(model, newStructure);

        if (cloneRNS)
        {
            std::vector<VirtualRobot::ModelNodeSetPtr> modelNodeSets;
            for (ModelNodeSetPtr rns : model->getModelNodeSets())
            {
                modelNodeSets.push_back(model->getModelNodeSet(rns->getName())->clone(r));
            }
        }

        if (cloneEEF)
        {
            // Copy end effectors
            for(auto &eef : model->getEndEffectors())
            {
                eef->clone(r);
            }
        }

        return r;
    }*/

    /*
    ModelPtr ModelFactory::cloneChangeStructure(const ModelPtr& model, const std::string& startNode, const std::string& endNode)
    {
        VR_ASSERT(model);

        if (!model->hasModelNode(startNode))
        {
            VR_ERROR << "No node with name " << startNode << endl;
            return ModelPtr();
        }

        if (!model->hasModelNode(endNode))
        {
            VR_ERROR << "No node with name " << endNode << endl;
            return ModelPtr();
        }

        if (!model->getModelNode(startNode)->hasChild(endNode, true))
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << endl;
            return ModelPtr();
        }

        std::vector<std::string> nodes;
        std::string currentNodeName = endNode;
        ModelNodePtr rn = model->getModelNode(currentNodeName);

        while (rn && !(rn->getName() == startNode))
        {
            currentNodeName = rn->getName();
            nodes.push_back(currentNodeName);
            rn = std::dynamic_pointer_cast<ModelNode>(rn->getParent());
        }

        if (!rn)
        {
            VR_ERROR << "No node " << endNode << " is not a child of " << startNode << endl;
            return ModelPtr();
        }

        nodes.push_back(startNode);
        //std::reverse(nodes.begin(), nodes.end());

        ModelFactory::modelStructureDef newStructure;
        newStructure.rootName = endNode;

        for (size_t i = 0; i < nodes.size() - 1; i++)
        {
            ModelFactory::modelNodeDef rnDef;
            rnDef.name = nodes[i];
            rnDef.children.push_back(nodes[i + 1]);
            rnDef.invertTransformation.push_back(true);
            newStructure.parentChildMapping.push_back(rnDef);
        }

        ModelFactory::modelNodeDef rnDef;
        rnDef.name = nodes[nodes.size() - 1];
        rnDef.invertTransformation.push_back(true);
        newStructure.parentChildMapping.push_back(rnDef);


        return ModelFactory::cloneChangeStructure(model, newStructure);
    }*/

/*
    bool ModelFactory::attach(const ModelPtr& model, SceneObjectPtr o, const ModelNodePtr& rn, const Eigen::Matrix4f& transformation)
    {
        if (!model || !o || !rn)
        {
            return false;
        }

        std::string name = o->getName();

        if (model->hasModelNode(name))
        {
            VR_WARNING << "RN with name " << name << " already present" << endl;
            return false;
        }

        ModelLink::Physics p = o->physics;
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

        auto rnf = ModelNodeFixedFactory::createInstance(NULL);
        ModelNodePtr newRN = rnf->createModelNode(model, name, v, c, 0, 0, 0, transformation, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);
        rn->attachChild(newRN);
        newRN->initialize(rn);
        return true;
    }

    bool ModelFactory::detach(const ModelPtr& model, const ModelNodePtr& rn)
    {
        if (!model || !rn || !rn->getParent())
        {
            return false;
        }

        if (!model->hasModelNode(rn))
        {
            return false;
        }

        rn->getParent()->detachChild(rn);
        model->deregisterModelNode(rn);
        return true;
    }

    ModelPtr ModelFactory::cloneChangeStructure(const ModelPtr& model, modelStructureDef& newStructure)
    {
        VR_ASSERT(model);

        if (!model->hasModelNode(newStructure.rootName))
        {
            VR_ERROR << "No root with name " << newStructure.rootName << endl;
            return ModelPtr();
        }

        std::map<std::string, ModelNodePtr> newNodes;
        ModelPtr newModel = createModel(model->getName(), model->getType() + "_restructured_" + newStructure.rootName);
        ModelNodePtr rn = model->getModelNode(newStructure.rootName);
        rn = rn->clone(newModel, false);
        newNodes[newStructure.rootName] = rn;
        newModel->setRootNode(newNodes[newStructure.rootName]);

        std::string nodeName;
        typedef std::map < ModelNodePtr,
                Eigen::Matrix4f,
                std::less<ModelNodePtr>,
                Eigen::aligned_allocator<std::pair<const int, Eigen::Matrix4f> > >
                NodeTransformationMapT;

        NodeTransformationMapT localTransformations;
        std::map<ModelNodePtr, VisualizationNodePtr> visuMap;
        std::map<ModelNodePtr, CollisionModelPtr> colMap;
        std::map<ModelNodePtr, ModelLink::Physics> physicsMap;
        std::map<ModelNodePtr, std::vector<SensorPtr> > sensorMap;
        std::map<ModelNodePtr, bool> directionInversion;

        for (size_t i = 0; i < newStructure.parentChildMapping.size(); i++)
        {
            if (!model->hasModelNode(newStructure.parentChildMapping[i].name))
            {
                VR_ERROR << "Error in parentChildMapping, no node with name " << newStructure.parentChildMapping[i].name << endl;
                return ModelPtr();
            }

            nodeName = newStructure.parentChildMapping[i].name;

            if (newNodes.find(nodeName) == newNodes.end())
            {
                rn = model->getModelNode(nodeName);
                rn = rn->clone(newModel, false);
                newNodes[nodeName] = rn;
            }

            ModelNodePtr parent = newNodes[newStructure.parentChildMapping[i].name];

            for (size_t j = 0; j < newStructure.parentChildMapping[i].children.size(); j++)
            {
                nodeName = newStructure.parentChildMapping[i].children[j];

                if (!model->hasModelNode(nodeName))
                {
                    VR_ERROR << "Error in parentChildMapping, no child node with name " << nodeName << endl;
                    return ModelPtr();
                }

                if (newNodes.find(nodeName) == newNodes.end())
                {
                    rn = model->getModelNode(nodeName);
                    rn = rn->clone(newModel, false);
                    newNodes[nodeName] = rn;
                }

                //children.push_back(newNodes[nodeName]);
                ModelNodePtr child = newNodes[nodeName];
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
                    ModelLink::Physics p = physicsMap[parent];
                    Eigen::Vector3f loc = p.localCoM;
                    l << loc(0), loc(1), loc(2), 1.0f;
                    physicsMap[parent].localCoM = (tr * l).head(3);

                    if (physicsMap.find(child) == physicsMap.end())
                    {
                        // no entry for child, set it to empty, may be overwritten later on
                        physicsMap[child] = ModelLink::Physics();
                    }

                    // exchange sensors
                    std::vector<SceneObjectPtr> childChildren = model->getModelNode(nodeName)->getChildren();

                    for (auto cc : childChildren)
                    {
                        SensorPtr cs = std::dynamic_pointer_cast<Sensor>(cc);

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
                    std::vector<SceneObjectPtr> childChildren = model->getModelNode(nodeName)->getChildren();

                    for (auto cc : childChildren)
                    {
                        SensorPtr cs = std::dynamic_pointer_cast<Sensor>(cc);

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
            std::map<ModelNodePtr, bool>::iterator inv_it = directionInversion.find(it->first);
            VR_ASSERT(inv_it != directionInversion.end());
            if (inv_it->second)
            {
                ModelNodeRevolutePtr rotJoint = std::dynamic_pointer_cast<ModelNodeRevolute>(it->first);
                if (rotJoint)
                {
                    rotJoint->jointRotationAxis *= -1.0f;
                }

                ModelNodePrismaticPtr prismaticJoint = std::dynamic_pointer_cast<ModelNodePrismatic>(it->first);
                if (prismaticJoint)
                {
                    prismaticJoint->jointTranslationDirection *= -1.0f;
                }
            }

            it++;
        }

        std::vector<ModelNodePtr> nodes = newModel->getModelNodes();

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

        newModel->getRootNode()->initialize();

        return newModel;
    }*/


    ModelPtr ModelFactory::clone(const ModelPtr& model, const std::string& name, const CollisionCheckerPtr& collisionChecker, float scaling)
    {
        THROW_VR_EXCEPTION_IF(!model, "NULL data");

		CollisionCheckerPtr colChecker = collisionChecker;
        if (!colChecker)
        {
			colChecker = model->getCollisionChecker();
        }

        VirtualRobot::ModelPtr result = model->extractSubPart(model->getRootNode(), model->getType(), name, true, true, colChecker, scaling);
        result->setGlobalPose(model->getGlobalPose());
        return result;
    }

    void ModelFactory::getChildNodes(const ModelNodePtr& nodeA, const ModelNodePtr& nodeExclude, std::vector<ModelNodePtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector < ModelNodePtr > children = nodeA->getChildNodes();
        std::vector<ModelNodePtr> childNodes;

        for (size_t i = 0; i < children.size(); i++)
        {
            ModelNodePtr cRN = children[i];
            //ModelNodePtr cRN = std::dynamic_pointer_cast<ModelNode>(c);

            if (cRN && cRN != nodeExclude)
            {
                appendNodes.push_back(cRN);
                getChildNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }
    /*
    void ModelFactory::getChildSensorNodes(const ModelNodePtr& nodeA, const ModelNodePtr& nodeExclude, std::vector<SensorPtr>& appendNodes)
    {
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        std::vector < SceneObjectPtr > children = nodeA->getChildren();
        std::vector<ModelNodePtr> childNodes;

        for (size_t i = 0; i < children.size(); i++)
        {
            SceneObjectPtr c = children[i];
            SensorPtr cS = std::dynamic_pointer_cast<Sensor>(c);
            ModelNodePtr cRN = std::dynamic_pointer_cast<ModelNode>(c);

            if (cS)
            {
                appendNodes.push_back(cS);
            }

            if (cRN && cRN != nodeExclude)
            {
                getChildSensorNodes(cRN, nodeExclude, appendNodes);
            }
        }
    }*/
    /*
    ModelNodePtr ModelFactory::accumulateTransformations(const ModelPtr& model, const ModelNodePtr& nodeA, const ModelNodePtr& nodeAClone, const ModelNodePtr& nodeB, Eigen::Matrix4f& storeTrafo)
    {
        THROW_VR_EXCEPTION_IF(!model, "NULL data");
        THROW_VR_EXCEPTION_IF(!nodeA, "NULL data");
        auto rnf = ModelNodeFixedFactory::createInstance(NULL);
        ModelLink::Physics p;
        VisualizationNodePtr v;
        CollisionModelPtr c;

        if (nodeA == nodeB)
        {
            ModelNodePtr newRN = rnf->createModelNode(model, nodeA->getName() + "_non_trafo", v, c, 0, 0, 0, Eigen::Matrix4f::Identity(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);
            return newRN;
        }

        std::vector<ModelNodePtr> childNodes;
        std::vector<SensorPtr> childSensorNodes;

        getChildNodes(nodeA, nodeB, childNodes);
        getChildSensorNodes(nodeA, nodeB, childSensorNodes);

        if (childNodes.size() == 0)
        {
            return ModelNodePtr();
        }

        storeTrafo = Eigen::Matrix4f::Identity();

        if (nodeA && nodeB)
        {
            Eigen::Matrix4f startPose = nodeA->getGlobalPose();
            Eigen::Matrix4f goalPose = nodeB->getParent()->getGlobalPose();
            storeTrafo = goalPose * startPose.inverse();
        }

        ModelNodePtr res = createUnitedModelNode(model, childNodes, nodeA, nodeAClone, Eigen::Matrix4f::Identity(), childSensorNodes);

        return res;
    }

    ModelNodePtr ModelFactory::createUnitedModelNode(const ModelPtr& model, const std::vector< ModelNodePtr >& nodes, const ModelNodePtr& parent, const ModelNodePtr& parentClone, const Eigen::Matrix4f& trafo, const std::vector<SensorPtr> &sensors)
    {
        THROW_VR_EXCEPTION_IF(!model, "NULL data");

        auto rnf = ModelNodeFixedFactory::createInstance(NULL);
        ModelLink::Physics p;
        VisualizationNodePtr v;
        CollisionModelPtr c;
        std::string name = "root";

        if (parentClone)
        {
            name = parentClone->getName() + "_FixedTrafo";
        }

        if (nodes.size() == 0)
        {
            ModelNodePtr newRN = rnf->createModelNode(model, name, v, c, 0, 0, 0, trafo, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);

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

        ModelNodePtr newRN = rnf->createModelNode(model, name, v, c, 0, 0, 0, trafo, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), p);

        newRN->initialize(parentClone);

        // attach sensors
        for (size_t i = 0; i < sensors.size(); i++)
        {
            SensorPtr s = sensors[i]->clone(newRN);
            Eigen::Matrix4f trafoToNewRN = parent?parent->getGlobalPose() * trafo:trafo;
            Eigen::Matrix4f t = trafoToNewRN.inverse() * sensors[i]->getGlobalPose();
            s->setModelNodeToSensorTransformation(t);
        }

        return newRN;
    }


    ModelPtr ModelFactory::cloneSubSet(const ModelPtr& model, const ModelNodeSetPtr& rns, const std::string& name)
    {
        THROW_VR_EXCEPTION_IF(!model, "NULL data");
        THROW_VR_EXCEPTION_IF(!rns, "NULL data");
        THROW_VR_EXCEPTION_IF(rns->getModel() != model, "Inconsitent data");

        std::vector< ModelNodePtr > nodes = rns->getAllModelNodes();
        THROW_VR_EXCEPTION_IF(nodes.size() == 0, "0 data");

        // ensure tcp is part of nodes
        //if (rns->getTCP() && !rns->hasModelNode(rns->getTCP()))
        //    nodes.push_back(rns->getTCP());

        // ensure kinemtic root is part of nodes
        if (rns->getKinematicRoot() && !rns->hasModelNode(rns->getKinematicRoot()))
            nodes.insert(nodes.begin(),rns->getKinematicRoot());

        ModelNodePtr startNode = rns->getKinematicRoot();
        if (!startNode)
            startNode = nodes[0];

        for (size_t i = 1; i < nodes.size(); i++)
        {
            ModelNodePtr a = nodes[i - 1];
            ModelNodePtr b = nodes[i];

            if (!a->hasChild(b, true))
            {
                std::stringstream ss;
                ss << "Node " << a->getName() << " is not parent of " << b->getName();
                THROW_VR_EXCEPTION(ss.str());
            }
        }

        // first create initial node
        std::vector< ModelNodePtr > initialNodes = startNode->getAllParents();
        // check for static nodes which are not parent of startNode
        std::vector< ModelNodePtr > allNodes = model->getModelNodes();
        for (size_t i=0;i<allNodes.size();i++)
        {
            ModelNodePtr rn = allNodes[i];
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

        ModelPtr result(new Model(name, model->getType()));

        Eigen::Matrix4f currentTrafo = Eigen::Matrix4f::Identity();

        if (startNode->getParent())
        {
            currentTrafo = startNode->getParent()->getGlobalPose();
        }

        // collect sensor nodes
        std::vector<SensorPtr> childSensorNodes;
        for (size_t i = 0; i < initialNodes.size(); i++)
        {
            ModelNodePtr rn = initialNodes[i];
            std::vector<SceneObjectPtr> c = rn->getChildren();
            for (size_t j = 0; j < c.size(); j++)
            {
                SensorPtr s = std::dynamic_pointer_cast<Sensor>(c[j]);
                if (s)
                    childSensorNodes.push_back(s);
            }
        }

        ModelNodePtr rootNode = createUnitedModelNode(result, initialNodes, ModelNodePtr(), ModelNodePtr(), Eigen::Matrix4f::Identity(), childSensorNodes);
        result->setRootNode(rootNode);
        ModelNodePtr currentParent = rootNode;
    
        for (size_t i = 0; i < nodes.size(); i++)
        {
            ModelNodePtr newNode = nodes[i]->clone(result, false, currentParent);
            Eigen::Matrix4f newTrafo = currentTrafo * newNode->getLocalTransformation();
            newNode->setLocalTransformation(newTrafo);
            currentParent = newNode;
            currentTrafo.setIdentity();

            ModelNodePtr secondNode;

            if (i < nodes.size() - 1)
            {
                secondNode = nodes[i + 1];
            } else
            {
                cout << "end";
            }

            ModelNodePtr newNodeFixed = accumulateTransformations(result, nodes[i], newNode, secondNode, currentTrafo);

            if (newNodeFixed)
            {
                currentParent = newNodeFixed;
            }
        }

        result->setGlobalPose(model->getGlobalPose());
        return result;
    }

    ModelPtr ModelFactory::cloneUniteSubsets(const ModelPtr& model, const std::string& name, const std::vector<std::string>& uniteWithAllChildren)
    {
        THROW_VR_EXCEPTION_IF(!model, "NULL data");
        if (uniteWithAllChildren.size() == 0)
            return ModelFactory::clone(model, model->getName());

        for (size_t i = 0; i < uniteWithAllChildren.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(!model->hasModelNode(uniteWithAllChildren[i]), "Could not find ModelNode in model");
        }


        ModelPtr result(new Model(name, model->getType()));


        ModelNodePtr currentNode = model->getRootNode();
        ModelNodePtr currentNodeClone = currentNode->clone(result, false);
        result->setRootNode(currentNodeClone);

        cloneRecursiveUnite(result, currentNode, currentNodeClone, uniteWithAllChildren);

        // clone RNS
        std::vector<ModelNodeSetPtr> rnsets = model->getModelNodeSets();
        for (size_t i = 0; i < rnsets.size(); i++)
        {
            ModelNodeSetPtr rns = rnsets[i];
            
            bool ok = true;
            for (size_t j = 0; j < uniteWithAllChildren.size(); j++)
            {
                ModelNodePtr rn = model->getModelNode(uniteWithAllChildren[j]);
                std::vector<ModelNodePtr> allChildren;
                rn->collectAllModelNodes(allChildren);
                for (size_t k = 0; k < allChildren.size(); k++)
                {
                    if (allChildren[k] == rn)
                        continue;
                    if (rns->hasModelNode(allChildren[k]->getName()))
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

        result->setGlobalPose(model->getGlobalPose());
        return result;
    }

    void ModelFactory::cloneRecursiveUnite(const ModelPtr& model, const ModelNodePtr& currentNode, const ModelNodePtr& currentNodeClone, const std::vector<std::string>& uniteWithAllChildren)
    {
        std::vector<SceneObjectPtr> c = currentNode->getChildren();


        for (size_t i = 0; i < c.size(); i++)
        {
            if (std::find(uniteWithAllChildren.begin(), uniteWithAllChildren.end(), c[i]->getName()) != uniteWithAllChildren.end())
            {
                ModelNodePtr currentRN = std::dynamic_pointer_cast<ModelNode>(c[i]);
                THROW_VR_EXCEPTION_IF(!currentRN, "Only RN allowed in list");
                ModelNodePtr currentRNClone = currentRN->clone(model, false, currentNodeClone);

                std::vector<ModelNodePtr> childNodes;
                std::vector<SensorPtr> childSensorNodes;

                getChildNodes(currentRN, ModelNodePtr(), childNodes);
                getChildSensorNodes(currentRN, ModelNodePtr(), childSensorNodes);

                if (childNodes.size() > 0 || childSensorNodes.size() > 0)
                {
                    ModelNodePtr res = createUnitedModelNode(model, childNodes, currentRN, currentRNClone, Eigen::Matrix4f::Identity(), childSensorNodes);
                    // res is automatically added
                }
            }
            else
            {
                ModelNodePtr currentRN = std::dynamic_pointer_cast<ModelNode>(c[i]);
                if (currentRN)
                {
                    ModelNodePtr currentRNClone = currentRN->clone(model, false, currentNodeClone);
                    cloneRecursiveUnite(model, currentRN, currentRNClone, uniteWithAllChildren);
                }
                else
                {
                    SensorPtr s = std::dynamic_pointer_cast<Sensor>(c[i]);
                    if (s)
                    {
                        s->clone(currentNodeClone);
                    } else
                        VR_INFO << "Skipping node " << c[i]->getName() << endl;
                }
            }
 
        }

    }*/

} // namespace VirtualRobot
