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
#pragma once

#include "Model.h"
#include "../Tools/MathTools.h"

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
                                           const std::vector<RobotNodePtr >& robotNodes,
                                           const std::map< RobotNodePtr, std::vector<std::string> >& childrenMap,
                                           const RobotNodePtr& rootNode)
        {
            return initializeModel(robot, robotNodes, childrenMap, rootNode);
        }
        static bool initializeModel(const ModelPtr& model,
                                    const std::vector<ModelNodePtr >& modelNodes,
                                    const std::map< ModelNodePtr, std::vector<std::string> >& childrenMap,
                                    const ModelNodePtr& rootNode);


        /*struct modelNodeDef
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
        };*/

        /*!
            Clones the model.
            \param model The model to clone
            \param name The new name
            \param collisionChecker Optional: A collision checker to which the model should be registered. If not set, the collision checker of the input model is used.
            \param scaling Scale the resulting model.
        */
        static ModelPtr clone(const ModelPtr& model, const std::string& name, const CollisionCheckerPtr& collisionChecker = CollisionCheckerPtr(), float scaling = 1.0f);


        /*!
         * \brief attach Attach an object to a model. The object is cloned.
         * \param model
         * \param o The object and its visualization model is cloned
         * \param mn The model node to which the object should be attached
         * \param transformation The RN to object transformation
         * \return true on succes
         */
        //static bool attach(const ModelPtr& model, ModelNodePtr o, const ModelNodePtr& mn, const Eigen::Matrix4f& transformation);

        //static bool detach(const ModelPtr& model, const ModelNodePtr& rn);


    protected:

        // some internal stuff
        static void getChildNodes(const ModelNodePtr& nodeA, const ModelNodePtr& nodeExclude, std::vector<ModelNodePtr>& appendNodes);

        // instantiation not allowed
        ModelFactory();
        virtual ~ModelFactory();
    };

    typedef ModelFactory RobotFactory;
}
