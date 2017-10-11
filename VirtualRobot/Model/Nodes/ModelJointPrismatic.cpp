#include "ModelJointPrismatic.h"
#include "../Model.h"

namespace VirtualRobot
{
    ModelJointPrismatic::ModelJointPrismatic(const ModelWeakPtr& model,
        const std::string& name,
        const Eigen::Matrix4f& staticTransformation,
        float jointLimitLo,
        float jointLimitHi,
        const Eigen::Vector3f& translationDirection,
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

    Eigen::Vector3f ModelJointPrismatic::getJointTranslationDirection(FramePtr coordSystem) const
    {
        if (coordSystem)
            return getJointTranslationDirection(coordSystem->getGlobalPose());
        else
            return getJointTranslationDirection(Eigen::Matrix4f::Identity());
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
        Eigen::Affine3f tmpT(Eigen::Translation3f((this->getJointValue() + getJointValueOffset()) * getJointTranslationDirectionJointCoordSystem()));
        return getStaticTransformation() * tmpT.matrix();
    }

    ModelNodePtr ModelJointPrismatic::_clone(ModelPtr newModel, float scaling)
    {
        Eigen::Matrix4f st = getStaticTransformation();
        ModelJointPrismaticPtr result(new ModelJointPrismatic(newModel, name, st, jointLimitLo, jointLimitHi, translationDirection, jointValueOffset));
        return result;
    }
}
