#include "ModelJointPrismatic.h"

namespace VirtualRobot
{
    ModelJointPrismatic::ModelJointPrismatic(const ModelWeakPtr& model,
        const std::string& name,
        Eigen::Matrix4f& staticTransformation,
        float jointLimitLo,
        float jointLimitHi,
        Eigen::Vector3f& translationDirection,
        float jointValueOffset)
            : ModelJoint(model, name, staticTransformation, jointLimitLo, jointLimitHi, jointValueOffset),
              translationDirection(translationDirection)
    {
    }

    ModelJointPrismatic::~ModelJointPrismatic()
    {
    }

    ModelNode::ModelNodeType ModelJointPrismatic::getType() const
    {
        return ModelNode::ModelNodeType::JointPrismatic;
    }

    Eigen::Vector3f ModelJointPrismatic::getJointTranslationDirection(CoordinatePtr coordSystem) const
    {
        if (coordSystem)
            return getJointTranslationDirection(coordSystem->getGlobalPose());
        else
            return getJointTranslationDirection();
    }

    Eigen::Vector3f ModelJointPrismatic::getJointTranslationDirection(const Eigen::Matrix4f& coordSystem) const
    {
        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = getJointTranslationDirectionJointCoordSystem();

        result4f = getGlobalPose() * result4f;

        result4f = coordSystem.inverse() * result4f;

        return result4f.segment(0, 3);
    }

    Eigen::Vector3f ModelJointPrismatic::getJointTranslationDirectionJointCoordSystem() const
    {
        return translationDirection;
    }

    Eigen::Matrix4f ModelJointPrismatic::getNodeTransformation() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        Eigen::Affine3f tmpT(Eigen::Translation3f(this->getJointValue()
                                                  + (getJointValueOffset())
                                                    * getJointTranslationDirectionJointCoordSystem()));
        return getStaticTransformation() * tmpT.matrix();
    }
}
