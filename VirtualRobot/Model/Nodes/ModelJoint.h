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
* @author     Adrian Knobloch
* @copyright  2016 Adrian Knobloch
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../../Tools/ConditionedLock.h"
#include "ModelNode.h"
#include "../../VirtualRobot.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ModelJoint : public ModelNode
    {
	public:
        /*!
         * Constructor with settings.
         *
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         * @param jointLimitLo The lower limit of this joint.
         * @param jointLimitHi The upper limit of this joint.
         * @param jointValueOffset The offset for the value of this joint.
         */
        ModelJoint(const ModelWeakPtr& model,
                   const std::string& name,
                   const Eigen::Matrix4f& localTransformation,
                   float jointLimitLo,
                   float jointLimitHi,
                   float jointValueOffset = 0.0f);

        /*!
         * Destructor.
         */
        virtual ~ModelJoint() override;

        //virtual ModelNodeType getType() const override;
        virtual bool isJoint() const override
        {
            return true;
        }
        virtual bool isTranslationalJoint() const override
        {
            return false;
        }
        virtual bool isRotationalJoint() const override
        {
            return false;
        }
        virtual bool isFixedJoint() const override
        {
            return false;
        }
        virtual bool isLink() const override
        {
            return false;
        }

        /*!
         * Set a joint value [rad/mm].
         * The internal matrices and visualizations are updated accordingly.
         * If you intend to update multiple joints, use \ref setJointValues for faster access.
         *
         * @param q The joint value in rad/mm.
         */
        virtual void setJointValue(float q);

        /*!
         * Set the joint value without updating the internal matrices.
         * After setting all joint values the transformations are calculated by calling \ref applyJointValues()
         * This method is used when multiple joints should be updated at once.
         *
         * @param q The joint value in rad/mm.
         */
        virtual void setJointValueNoUpdate(float q);

        /*!
         * Get the value [rad/mm] of this joint.
         *
         * @return the joint value in rad/mm.
         */
        virtual float getJointValue() const;

        virtual void copyPoseFrom(const ModelNodePtr& other) override;

        /*!
         * Checks if jointValue is within joint limits. If verbose is set a warning is printed.
         *
         * @param jointValue The value to check.
         * @param verbose If set to true, a warning is printed, if the value does not respect the limits.
         */
        bool checkJointLimits(float jointValue, bool verbose = false) const;

        /*!
         * Checks if jointValue is within joint limits. If not jointValue is adjusted.
         *
         * @param jointValue The value to check and adjust.
         */
        void respectJointLimits(float& jointValue) const;

        /*!
         * Set joint limits [rad/mm].
         *
         * @param lo The lower limit in rad/mm.
         * @param hi The higher limit in rad/mm.
         */
        virtual void setJointLimits(float lo, float hi);

        /*!
         * Get the offset of the joint value.
         *
         * @return The offset.
         */
        virtual float getJointValueOffset() const;

        /*!
         * Get the upper joint limit.
         *
         * @return The upper joint limit in rad/mm.
         */
        virtual float getJointLimitHigh() const;

        /*!
         * Get the lower joint limit.
         *
         * @return The lower joint limit in rad/mm.
         */
        virtual float getJointLimitLow() const;

        /*!
         * Set maximum velocity of this joint in rad/s or m/s.
         * To disbale max. velocity set to -1.0f.
         *
         * @param maxVel The new maximum velocity in rad/s or m/s.
         */
        virtual void setMaxVelocity(float maxVel);

        /*!
         * Set maximum acceleration pf this joint in rad/s^2 or m/s^2.
         * To disbale max. acceleration set to -1.0f.
         *
         * @param maxVel The new maximum acceleration in rad/s^2 or m/s^2.
         */
        virtual void setMaxAcceleration(float maxAcc);

        /*!
         * Set maximum torque pf this joint in Nm.
         * To disbale max. torque set to -1.0f.
         *
         * @param maxVel The new maximum torque in Nm.
         */
        virtual void setMaxTorque(float maxTo);

        /*!
         * Maximum velocity in rad/s or m/s.
         * Returns -1.0f if no maximum value is set.
         *
         * @return The maximum velocity in rad/s or m/s, or -1.0f if no value is set.
         */
        virtual float getMaxVelocity() const;

        /*!
         * Maximum acceleration in rad/s^2 or m/s^2.
         * Returns -1.0f if no maximum value is set.
         *
         * @return The maximum acceleration in rad/s^2 or m/s^2, or -1.0f if no value is set.
         */
        virtual float getMaxAcceleration() const;

        /*!
         * Maximum torque in Nm.
         * Returns -1.0f if no maximum value is set.
         *
         * @return The maximum torque in Nm, or -1.0f if no value is set.
         */
        virtual float getMaxTorque() const;

        /*!
         * Automatically propagate the joint value to another joint.
         *
         * @param jointName The name of the other joint. Must be part of the same robot.
         * @param factor The propagated joint value is scaled according to this value.
         *               If this factor is 0.0f the propagation is deleted.
         */
        virtual void propagateJointValue(const std::string& jointName, float factor = 1.0f);

        virtual void updatePoseInternally(bool updateChildren, bool updateAttachments) override;

        /**
         * @param limitless wheter this node has joint limits or not.
         */
        virtual void setLimitless(bool limitless);
        virtual bool isLimitless() const;

        /**
         * @param target the target joint value in [rad]
         * @return the signed distance between current and target joint values in [rad].
         *         If the given target value violates joint limits or this robotnode is not a joint, 0.0f is returned instead.
         */
        float getDelta(float target) const;

    protected:
        ModelJoint();
        float jointValue;           //!< The joint value
        float jointValueOffset;
        float jointLimitLo;
        float jointLimitHi;
        float maxVelocity;          //!< given in m/s
        float maxAcceleration;      //!< given in m/s^2
        float maxTorque;            //!< given in Nm

        bool limitless;

        std::map< std::string, float > propagatedJointValues;
    };
}
