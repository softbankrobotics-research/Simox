
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
                                       const std::vector<ModelNodePtr >& modelNodes,
                                       const std::map< ModelNodePtr, std::vector<std::string> >& childrenMap,
                                       const ModelNodePtr& rootNode
                                      )
    {
        VR_ASSERT(model);
        bool result = true;

        // check for root
        auto iter = modelNodes.begin();
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
        ModelNodePtr currentNode = rootNode;
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


/*
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
*/
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

  */

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

} // namespace VirtualRobot
