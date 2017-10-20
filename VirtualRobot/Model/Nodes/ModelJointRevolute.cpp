#include "ModelJointRevolute.h"

namespace VirtualRobot
{
    ModelJointRevolute::ModelJointRevolute(const ModelWeakPtr& model,
        const std::string& name,
        const Eigen::Matrix4f& staticTransformation,
        float jointLimitLo,
        float jointLimitHi,
        const Eigen::Vector3f& axis,
        float jointValueOffset)
            : ModelJoint(model, name, staticTransformation, jointLimitLo, jointLimitHi, jointValueOffset),
              axis(axis)
    {
    }

    ModelJointRevolute::~ModelJointRevolute()
    {
    }

    ModelNode::ModelNodeType ModelJointRevolute::getType() const
    {
        return ModelNode::ModelNodeType::JointRevolute;
    }

    Eigen::Vector3f ModelJointRevolute::getJointRotationAxis(FramePtr coordSystem) const
    {
        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = getJointRotationAxisInJointCoordSystem();
        result4f = globalPose * result4f;

        if (coordSystem)
        {
            //res = coordSystem->toLocalCoordinateSystem(res);
            result4f = coordSystem->getGlobalPose().inverse() * result4f;
        }

        return result4f.block(0, 0, 3, 1);
    }

    Eigen::Vector3f ModelJointRevolute::getJointRotationAxisInJointCoordSystem() const
    {
        return axis;
    }

    Eigen::Matrix4f ModelJointRevolute::getNodeTransformation() const
    {
        Eigen::Matrix4f tmpRotMat = Eigen::Matrix4f::Identity();
        tmpRotMat.block(0, 0, 3, 3) = Eigen::AngleAxisf(jointValue + jointValueOffset,
                                                        getJointRotationAxisInJointCoordSystem()).matrix();
        return getStaticTransformation() * tmpRotMat;
    }

    ModelNodePtr ModelJointRevolute::_clone(ModelPtr newModel, float scaling)
    {
        Eigen::Matrix4f st = getStaticTransformation();
        ModelJointRevolutePtr result(new ModelJointRevolute(newModel, name, st, jointLimitLo, jointLimitHi, axis, jointValueOffset));
        result->setLimitless(limitless);
        return result;
    }
}
