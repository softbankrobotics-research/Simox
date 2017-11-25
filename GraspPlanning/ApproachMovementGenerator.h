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
* @package    GraspPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _GRASP_PLANNING_APPROACH_MOVEMENT_GENERATOR_H_
#define _GRASP_PLANNING_APPROACH_MOVEMENT_GENERATOR_H_

#include "GraspPlanning.h"
#include <VirtualRobot/Model/Model.h>
#include <VirtualRobot/Model/Model.h>
#include <vector>
#include <string>
#include <Eigen/Core>

namespace GraspPlanning
{

    /*!
    *
    * An interface for generating approach movements of an end-effector toward an object resulting in grasp hypothesis.
    */
    class GRASPPLANNING_IMPORT_EXPORT ApproachMovementGenerator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            To generate approach movements an object and an end effector has to be specified.
            Internally a clone of the EEF is used.
            \param object The object must contain exactly one link with a collision model.
            \param eef The eef.
            \param graspPreshape An optional preshape that can be used in order to "open" the eef.
        */
        ApproachMovementGenerator(VirtualRobot::ModelPtr object, VirtualRobot::EndEffectorPtr eef, std::string graspPres = "");

        //! destructor
        virtual ~ApproachMovementGenerator();

        //! Creates a new pose for approaching
        virtual Eigen::Matrix4f createNewApproachPose() = 0;

        //! Applies a random grasp hypothesis to the cloned EEF
        virtual bool setEEFToRandomApproachPose();

        //! This robot is moved around
        const VirtualRobot::RobotPtr& getEEFRobotClone();

        //! move EEF to pose (uses coord system of GCP)
        bool setEEFPose(const Eigen::Matrix4f& pose);

        //! update pose of EEF (GCP)
        bool updateEEFPose(const Eigen::Matrix4f& deltaPose);
        bool updateEEFPose(const Eigen::Vector3f& deltaPosition);

        //! get pose of EEF (coord system of GCP)
        Eigen::Matrix4f getEEFPose();

        std::string getGCPJoint();

        const VirtualRobot::ModelPtr& getObject();

        /*!
            This is the cloned eef!
        */
        const VirtualRobot::EndEffectorPtr& getEEF();
        const VirtualRobot::EndEffectorPtr& getEEFOriginal();

        const Eigen::Vector3f& getApproachDirGlobal();

        const std::string& getName();

        // opens hand by setting the preshape (if defined) or just opening the actors in case no preshape has been defined
        virtual void openHand();

protected:
        VirtualRobot::ModelPtr object;

        VirtualRobot::TriMeshModelPtr objectModel;
        VirtualRobot::EndEffectorPtr eef;

        Eigen::Vector3f approachDirGlobal;

        //! This robot is moved around
        VirtualRobot::RobotPtr eefRobot;
        VirtualRobot::EndEffectorPtr eef_cloned;

        std::string name;
        std::string graspPreshape;
    };

}

#endif /* __APPROACH_MOVEMENT_GENERATOR_H__ */
