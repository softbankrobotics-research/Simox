/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ModelFactory_h_
#define _VirtualRobot_ModelFactory_h_

#include "VirtualRobot.h"
#include "MathTools.h"

#include <string>
#include <vector>
#include <map>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelFactory
    {
    public:
        /*!
        Creates an empty model.
        */
        static inline RobotPtr createRobot(const std::string& name, const std::string& type = "")
        {
            return createModel(name, type);
        }
        static ModelPtr createModel(const std::string& name, const std::string& type = "");

        /*!
            Initializes Model and all ModelNodes.
            \param modelNodes All nodes of the model. Must contain rootNode.
            \param childrenMap Parent-child relations are built according to this data.
            \param rootNode The root.
        */
        static inline bool initializeRobot(const RobotPtr& robot,
                                           std::vector<RobotNodePtr >& robotNodes,
                                           const std::map< RobotNodePtr, std::vector<std::string> >& childrenMap,
                                           const RobotNodePtr& rootNode)
        {
            return initializeModel(robot, robotNodes, childrenMap, rootNode);
        }
        static bool initializeModel(const ModelPtr& model,
                                    std::vector<ModelNodePtr >& modelNodes,
                                    const std::map< ModelNodePtr, std::vector<std::string> >& childrenMap,
                                    const ModelNodePtr& rootNode);


        struct modelNodeDef
        {
            std::string name;
            std::vector<std::string> children;
            // used to mark children whose transformation should be inverted
            std::vector<bool> invertTransformation;
        };

        struct modelStructureDef
        {
            std::string rootName;
            std::vector<modelNodeDef> parentChildMapping;
        };

        /*!
            Clones the model.
            \param model The model to clone
            \param name The new name
            \param collisionChecker Optional: A collision checker to which the model should be registered. If not set, the collision checker of the input model is used.
            \param scaling Scale the resulting model.
        */
        static ModelPtr clone(const ModelPtr& model, const std::string& name, const CollisionCheckerPtr& collisionChecker = CollisionCheckerPtr(), float scaling = 1.0f);

        /*!
        Clones the model, but only leave the defined joints active. ALl other joints are accumulated and set to one model which is fixed (may result in faster updates)
        \param model The model to clone
        \param rns The model node set of active joints. The joints must be given as an ordered set, i.e. node i must be located before node i+1 in the kinematic structure of the model.
        \param name The new name
        */
        static ModelPtr cloneSubSet(const ModelPtr& model, const ModelNodeSetPtr& rns, const std::string& name);

        /*!
            Creates a model clone with reduced structure.
            \param model The model to clone.
            \param uniteWithAllChildren List of ModelNodeNames. Each listed model ndoe is united with all of its children to one fixed ModelNode.
                                        This means that all related coordinate systems and joints will not be present in the clone. The visualizations are united.
        */
        static ModelPtr cloneUniteSubsets(const ModelPtr& model, const std::string& name, const std::vector<std::string>& uniteWithAllChildren);

        /*!
            Creates a clone with changed structure, so that the given model node is the new root of the resulting kinematic tree.
        */
        static ModelPtr cloneInversed(const ModelPtr& model, const std::string& newRootName, bool cloneRNS = true, bool cloneEEF = true);

        /*!
            Chenge the structure of the clone according to the given defintion.
        */
        static ModelPtr cloneChangeStructure(const ModelPtr& model, modelStructureDef& newStructure);

        /*! Clone kinematic chain and reverse direction.
         *
         * \param startNode Name of the start node of the original kinematic chain.
         * \param endNode Name of the end node of the original kinematic chain. Will be the new root.
         */
        static ModelPtr cloneChangeStructure(const ModelPtr& model, const std::string& startNode, const std::string& endNode);

        /*!
         * \brief attach Attach an object to a model. The object is cloned.
         * \param model
         * \param o The object and its visualization model is cloned
         * \param mn The model node to which the object should be attached
         * \param transformation The RN to object transformation
         * \return true on succes
         */
        static bool attach(const ModelPtr& model, SceneObjectPtr o, const ModelNodePtr& mn, const Eigen::Matrix4f& transformation);

        static bool detach(const ModelPtr& model, const ModelNodePtr& rn);


    protected:

        // some internal stuff
        static ModelNodePtr createUnitedModelNode(const ModelPtr& model, const std::vector< ModelNodePtr >& nodes, const ModelNodePtr& parent, const ModelNodePtr& parentClone, const Eigen::Matrix4f& trafo, const std::vector<SensorPtr> &sensors);
        static ModelNodePtr accumulateTransformations(const ModelPtr& model, const ModelNodePtr& nodeA, const ModelNodePtr& nodeAClone, const ModelNodePtr& nodeB, Eigen::Matrix4f& storeTrafo);
        static void getChildNodes(const ModelNodePtr& nodeA, const ModelNodePtr& nodeExclude, std::vector<ModelNodePtr>& appendNodes);
        static void getChildSensorNodes(const ModelNodePtr& nodeA, const ModelNodePtr& nodeExclude, std::vector<SensorPtr>& appendNodes);

        static void cloneRecursiveUnite(const ModelPtr& model, const ModelNodePtr& currentNode, const ModelNodePtr& currentNodeClone, const std::vector<std::string>& uniteWithAllChildren);

        // instantiation not allowed
        ModelFactory();
        virtual ~ModelFactory();
    };

    typedef ModelFactory RobotFactory;
}

#endif // _VirtualRobot_ModelFactory_h_
