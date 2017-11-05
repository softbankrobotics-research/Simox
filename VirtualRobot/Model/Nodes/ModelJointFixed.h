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
#ifndef _VirtualRobot_ModelJointFixed_h_
#define _VirtualRobot_ModelJointFixed_h_

#include "ModelJoint.h"

namespace VirtualRobot
{
    class ModelJointFixed : public ModelJoint
    {
    public:
        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         */
        ModelJointFixed(const ModelWeakPtr& model,
                        const std::string& name,
                        const Eigen::Matrix4f& localTransformation);

        /*!
         * Destructor.
         */
        virtual ~ModelJointFixed();

        virtual ModelNodeType getType() const override;

        /*!
         * Disable changing of joint value.
         *
         * @param q This does nothing.
         */
        virtual void setJointValue(float q) override;

        /*!
         * Disable changing of joint value.
         *
         * @param q This does nothing.
         */
        virtual void setJointValueNoUpdate(float q) override;

        /*!
         * Disable changing of joint limits.
         *
         * @param lo This does nothing.
         * @param hi This does nothing.
         */
        virtual void setJointLimits(float lo, float hi) override;

        /*!
         * Disable changing of max velocity.
         *
         * @param maxVel This does nothing.
         */
        virtual void setMaxVelocity(float maxVel) override;

        /*!
         * Disable changing of max acceleration.
         *
         * @param maxAcc This does nothing.
         */
        virtual void setMaxAcceleration(float maxAcc) override;

        /*!
         * Disable changing of max torque.
         *
         * @param maxTo This does nothing.
         */
        virtual void setMaxTorque(float maxTo) override;

        /*!
         * Creates an XML string that defines the ModelNode. Filenames of all visualization models are set to modelPath/RobotNodeName_visu and/or modelPath/RobotNodeName_colmodel.
         *
         * \see ModelIO::saveXML.
         *
         * @param basePath TODO: Documentation
         * @param modelPathRelative TODO: Documentation
         * @param storeAttachments If set to true, all attachments are stored in the XML.
         *
         * @return The generated XML string.
         */
        virtual std::string toXML(const std::string& basePath, const std::string& modelPathRelative = "models", bool storeAttachments = true) override;


    protected:
        ModelJointFixed();
        virtual ModelNodePtr _clone(ModelPtr newModel, float scaling = 1.0f) override;

    };
}


#endif // _VirtualRobot_ModelJointFixed_h_
