#include "ModelJointPrismatic.h"

namespace VirtualRobot
{
    ModelJointPrismatic::ModelJointPrismatic(ModelWeakPtr model,
                                             const std::string& name,
                                             Eigen::Matrix4f& staticTransformation,
                                             float jointLimitLo,
                                             float jointLimitHi,
                                             float jointValueOffset,
                                             Eigen::Vector3f& translationDirection)
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

    Eigen::Vector3f ModelJointPrismatic::getJointTranslationDirection(Eigen::Matrix4f coordSystem) const
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