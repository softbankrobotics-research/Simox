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
* @author     Peter Kaiser
* @copyright  2018, Peter Kaiser
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"

#include "RobotNode.h"
#include "../RobotFactory.h"

#include "../XML/exprtk.hpp"

#include <Eigen/Core>

#include <string>
#include <vector>


namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeMeta : public RobotNode
    {
    public:
        friend class RobotFactory;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
        */
        RobotNodeMeta(RobotWeakPtr rob,                                     //!< The robot
                           const std::string& name,                            //!< The name
                           float jointLimitLo,                                 //!< lower joint limit
                           float jointLimitHi,                                 //!< upper joint limit
                           const Eigen::Matrix4f& preJointTransform,           //!< This transformation is applied before the translation of the joint is done
                           VisualizationNodePtr visualization = VisualizationNodePtr(),    //!< A visualization model
                           CollisionModelPtr collisionModel = CollisionModelPtr(),         //!< A collision model
                           float jointValueOffset = 0.0f,                                  //!< An offset that is internally added to the joint value
                           const SceneObject::Physics& p = SceneObject::Physics(),         //!< physics information
                           CollisionCheckerPtr colChecker = CollisionCheckerPtr(),         //!< A collision checker instance (if not set, the global col checker is used)
                           RobotNodeType type = Generic);
        RobotNodeMeta(RobotWeakPtr rob,                                     //!< The robot
                           const std::string& name,                            //!< The name
                           float jointLimitLo,                                 //!< lower joint limit
                           float jointLimitHi,                                 //!< upper joint limit
                           float a,                                            //!< Use DH parameters for setting up this RobotNode
                           float d,                                            //!< Use DH parameters for setting up this RobotNode
                           float alpha,                                        //!< Use DH parameters for setting up this RobotNode
                           float theta,                                        //!< Use DH parameters for setting up this RobotNode
                           VisualizationNodePtr visualization = VisualizationNodePtr(),    //!< A visualization model
                           CollisionModelPtr collisionModel = CollisionModelPtr(),         //!< A collision model
                           float jointValueOffset = 0.0f,                                  //!< An offset that is internally added to the joint value
                           const SceneObject::Physics& p = SceneObject::Physics(),         //!< physics information
                           CollisionCheckerPtr colChecker = CollisionCheckerPtr(),         //!< A collision checker instance (if not set, the global col checker is used)
                           RobotNodeType type = Generic);
        /*!
        */
        ~RobotNodeMeta() override;

        bool initialize(SceneObjectPtr parent = SceneObjectPtr(), const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>()) override;

        void setDependencies(const std::map<std::string, std::string> dependency_functions);

        /*!
        Print status information.
        */
        void print(bool printChildren = false, bool printDecoration = true) const override;

        bool isMetaJoint() const override;

        void setJointValueNoUpdate(float q) override;

    protected:

        //! Checks if nodeType constraints are fulfilled. Otherwise an exception is thrown. Called on initialization.
        void checkValidRobotNodeType() override;

        RobotNodeMeta() {};

        RobotNodePtr _clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling) override;
        /*!
            Derived classes add custom XML tags here
        */
        std::string _toXML(const std::string& modelPath) override;

        bool compileDependencyFunctions();

    protected:
        bool visuScaling;
        Eigen::Vector3f visuScaleFactor;
        Eigen::Vector3f unscaledLocalCoM;

        std::map<std::string, std::string> dependencyFunctions;
        std::map<RobotNodePtr, exprtk::expression<float>> compiledDependencyFunctions;
        exprtk::symbol_table<float> dependencySymbolTable;
    };

    typedef boost::shared_ptr<RobotNodeMeta> RobotNodeMetaPtr;

} // namespace VirtualRobot

