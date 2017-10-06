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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _SimDynamics_DynamicsRobot_h_
#define _SimDynamics_DynamicsRobot_h_

#include "../SimDynamics.h"
#include "DynamicsObject.h"
#include "DynamicsUtils.h"
#include <VirtualRobot/Model/Model.h>
//#include <VirtualRobot/Nodes/Sensor.h>


namespace SimDynamics
{
    class SIMDYNAMICS_IMPORT_EXPORT DynamicsRobot
    {
        friend class DynamicsEngine;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
        */
        DynamicsRobot(VirtualRobot::RobotPtr rob);

        /*!
        */
        virtual ~DynamicsRobot();

        std::string getName() const;

        VirtualRobot::RobotPtr getRobot()
        {
            return robot;
        }

        bool hasDynamicsRobotNode(VirtualRobot::ModelLinkPtr node);
        std::vector<DynamicsObjectPtr> getDynamicsRobotNodes();

        /*!
            Returns dynamic model of node.
            An empty DynamicsObjectPtr is returned in case no dynamic version has been created so far.
        */
        DynamicsObjectPtr getDynamicsRobotNode(VirtualRobot::ModelLinkPtr node);
        DynamicsObjectPtr getDynamicsRobotNode(const std::string &nodeName);


        /*!
            Enable joint actuation for given node.
        */
        virtual void actuateNode(VirtualRobot::ModelJointPtr node, double jointValue, double jointVelocity);
        virtual void actuateNode(VirtualRobot::ModelJointPtr node, double jointValue);
        virtual void actuateNodeVel(VirtualRobot::ModelJointPtr node, double jointVelocity);
        virtual void actuateNodeTorque(VirtualRobot::ModelJointPtr node, double jointTorque);
        virtual void actuateNode(const std::string& node, double jointValue);
        virtual void actuateNodeVel(const std::string& node, double jointVelocity);
        virtual void actuateNodeTorque(const std::string& node, double jointTorque);
        virtual void disableNodeActuation(VirtualRobot::ModelJointPtr node);
        virtual bool isNodeActuated(VirtualRobot::ModelJointPtr node);
        virtual double getNodeTarget(VirtualRobot::ModelJointPtr node);
        virtual void enableActuation(ActuationMode mode);
        virtual void disableActuation();

        /*!
            Usually this method is called by the framework in every tick to perform joint actuation.
            \param dt Timestep
        */
        virtual void actuateJoints(double dt);
        virtual void updateSensors(double) {}

        /*!
         * \brief updateVisualization Update the poses of the visualization models.
         */
        virtual void updateVisualization();

        // experimental...
        virtual void ensureKinematicConstraints();

        // We do not allow to re-adjust the robot.
        // The position of the robot is queried once on construction.
        // Then the physics simulation takes over.
        //virtual void setPosition(const Eigen::Vector3f &posMM);
        //virtual void setPose(const Eigen::Matrix4f &pose);


        virtual double getJointAngle(VirtualRobot::ModelJointPtr rn);
        virtual double getJointSpeed(VirtualRobot::ModelJointPtr rn);
        virtual double getJointTargetSpeed(VirtualRobot::ModelJointPtr rn);

        virtual Eigen::Matrix4f getComGlobal(const VirtualRobot::ModelLinkPtr& rn);

        virtual Eigen::Vector3f getComGlobal(const VirtualRobot::LinkSetPtr& bodies);
        virtual Eigen::Vector3f getComVelocityGlobal(const VirtualRobot::LinkSetPtr& bodies);

        virtual Eigen::Vector3f getLinearMomentumGlobal(const VirtualRobot::LinkSetPtr& set);
        virtual Eigen::Vector3f getAngularMomentumGlobal(const VirtualRobot::LinkSetPtr& set);
        virtual Eigen::Vector3f getAngularMomentumLocal(const VirtualRobot::LinkSetPtr& set);

        virtual void setGlobalPose(const Eigen::Matrix4f& gp);

        //! If set, all actions are protected with this mutex
        virtual void setMutex(std::shared_ptr <std::recursive_mutex> engineMutexPtr);

        //! can be used to access the internal controllers
        std::map<VirtualRobot::ModelJointPtr, VelocityMotorController>& getControllers();

        typedef std::shared_ptr< std::unique_lock<std::recursive_mutex> > MutexLockPtr;
        /*!
            This lock can be used to protect data access. It locks the mutex until deletion.
            If no mutex was specified, an empty lock will be returned which does not protect the engine calls (this is the standard behavior).
            \see setMutex

            Exemplary usage:
            {
                MutexLockPtr lock = engine->getScopedLock();
                // now the mutex is locked

                // access data
                // ...

            } // end of scope -> lock gets deleted and mutex is released automatically
        */
        MutexLockPtr getScopedLock();

        void setPIDParameters(float p, float i, float d);

        /**
         * @brief Will enable or disable all collisions between the bodies of this robot.
         * @param enable
         * @note If true, this will overwrite all ignored collisions, that were individually set.
         */
        virtual void enableSelfCollisions(bool enable);

    protected:


        virtual void setPoseNonActuatedRobotNodes(){}

        virtual void createDynamicsNode(VirtualRobot::ModelLinkPtr node);

        //! creates a link and attaches object to internal data structure
        virtual bool attachObject(const std::string& nodeName, DynamicsObjectPtr object);
        virtual bool detachObject(DynamicsObjectPtr object);


        struct robotNodeActuationTarget
        {
            robotNodeActuationTarget()
                : jointValueTarget(0)
                , jointVelocityTarget(0)
                , jointTorqueTarget(0)
            {
                actuation.mode = 0;
            }
            double jointValueTarget;
            double jointVelocityTarget;
            double jointTorqueTarget;
            VirtualRobot::ModelJointPtr node;
            ActuationMode actuation;
        };

        std::map<VirtualRobot::ModelJointPtr, robotNodeActuationTarget> actuationTargets;
        std::map<VirtualRobot::ModelJointPtr, VelocityMotorController> actuationControllers;

        VirtualRobot::RobotPtr robot;

        //std::vector<VirtualRobot::SensorPtr> sensors;

		std::vector<VirtualRobot::ModelJointPtr> jointNodes;
		std::vector<VirtualRobot::ModelLinkPtr> linkNodes;
		std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr> dynamicRobotNodes;
        float PID_p = 10.f;
        float PID_i = 0.f;
        float PID_d = 0.f;

        std::shared_ptr <std::recursive_mutex> engineMutexPtr;
    };

    typedef std::shared_ptr<DynamicsRobot> DynamicsModelPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsRobot_h_
