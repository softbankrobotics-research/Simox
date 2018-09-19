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

#include "ModelJoint.h"

namespace VirtualRobot
{
    class ModelJointPrismatic : public ModelJoint
    {
    public:
        /*!
         * Constructor with settings.
         *
         * @param model A pointer to the Model, which uses this Node.
         * @param name The name of this ModelNode. This name must be unique for the Model.
         * @param localTransformation The transformation from the parent of this node to this node.
         * @param jointLimitLo The lower limit of this joint.
         * @param jointLimitHi The upper limit of this joint.
         * @param jointValueOffset The offset for the value of this joint.
         * @param translationDirection The move direction of this joint.
         */
        ModelJointPrismatic(const ModelWeakPtr& model,
            const std::string& name,
            const Eigen::Matrix4f& localTransformation,
            float jointLimitLo,
            float jointLimitHi,
            const Eigen::Vector3f& translationDirection,
            float jointValueOffset = 0.0f);

        /*!
         * Destructor.
         */
        virtual ~ModelJointPrismatic() override;

        virtual ModelNodeType getType() const override;

        /*!
         * In global coord system.
         *
         * @param coordSystem The coordinate system to get the direction in.
         */
        Eigen::Vector3f getJointTranslationDirection(const Eigen::Matrix4f& coordSystem = Eigen::Matrix4f::Identity()) const;
        Eigen::Vector3f getJointTranslationDirection(FramePtr coordSystem) const;

        /*!
         * This is the original joint axis, without any transformations applied.
         *
         * @return The translation direction in the local coordinate system of this node.
         */
        Eigen::Vector3f getJointTranslationDirectionJointCoordSystem() const;

        virtual Eigen::Matrix4f getNodeTransformation() const override;

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
        ModelJointPrismatic(){}
        virtual ModelNodePtr _clone(ModelPtr newModel, float scaling = 1.0f) override;
        Eigen::Vector3f translationDirection;
    };
}
