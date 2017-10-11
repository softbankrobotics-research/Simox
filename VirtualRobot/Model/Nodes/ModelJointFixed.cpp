#include "ModelJointFixed.h"

namespace VirtualRobot
{
    ModelJointFixed::ModelJointFixed(const ModelWeakPtr& model, const std::string& name, const Eigen::Matrix4f& localTransformation)
            : ModelJoint(model, name, localTransformation, 0, 0, 0.0f)
    {
    }

    ModelJointFixed::~ModelJointFixed()
    {
    }

    ModelNode::ModelNodeType ModelJointFixed::getType() const
    {
        return ModelNode::ModelNodeType::JointFixed;
    }

    void ModelJointFixed::setJointValue(float)
    {
        // do nothing
    }

    void ModelJointFixed::setJointValueNoUpdate(float)
    {
        // do nothing
    }

    void ModelJointFixed::setJointLimits(float, float)
    {
        // do nothing
    }

    void ModelJointFixed::setMaxVelocity(float)
    {
        // do nothing
    }

    void ModelJointFixed::setMaxAcceleration(float)
    {
        // do nothing
    }

    void ModelJointFixed::setMaxTorque(float)
    {
        // do nothing
    }

    ModelNodePtr ModelJointFixed::_clone(ModelPtr newModel, float scaling)
    {
        ModelJointFixedPtr result(new ModelJointFixed(newModel, name, getStaticTransformation()));
        return result;
    }
}
