#include "ModelJoint.h"
#include "../../VirtualRobotException.h"

namespace VirtualRobot
{
    ModelJoint::ModelJoint(const ModelWeakPtr& model,
                           const std::string& name,
                           const Eigen::Matrix4f& localTransformation,
                           float jointLimitLo,
                           float jointLimitHi,
                           float jointValueOffset) : ModelNode(model, name, localTransformation),
                                                     jointValue(0),
                                                     jointValueOffset(jointValueOffset),
                                                     jointLimitLo(jointLimitLo),
                                                     jointLimitHi(jointLimitHi),
                                                     maxVelocity(-1.0f),
                                                     maxAcceleration(-1.0f),
                                                     maxTorque(-1.0f),
                                                     limitless(false)
    {
    }

    ModelJoint::~ModelJoint()
    {
    }
	/*
    ModelNode::ModelNodeType ModelJoint::getType() const
    {
        return ModelNode::ModelNodeType::Joint;
    }*/

    void ModelJoint::setJointValue(float q)
    {
        setJointValueNoUpdate(q);
        updatePose(true, true);
    }

    void ModelJoint::setJointValueNoUpdate(float q)
    {
        VR_ASSERT_MESSAGE((!std::isnan(q) && !std::isinf(q)) , "Not a valid number...");
        WriteLockPtr w = getModel()->getWriteLock();

        if (limitless)
        {
            while (q > jointLimitHi) q -= float(2.0f * M_PI);
            while (q < jointLimitLo) q += float(2.0f * M_PI);
        }
        respectJointLimits(q);

        jointValue = q;
    }

    float ModelJoint::getJointValue() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return jointValue;
    }

    void ModelJoint::copyPoseFrom(const ModelNodePtr &other)
    {
        VR_ASSERT(ModelNode::checkNodeOfType(other, ModelNode::Joint));
        jointValue = std::static_pointer_cast<ModelJoint>(other)->getJointValue();
        ModelNode::copyPoseFrom(other);
    }

    bool ModelJoint::checkJointLimits(float jointValue, bool verbose) const
    {
        bool res = true;

        if (jointValue < getJointLimitLow())
        {
            res = false;
        }

        if (jointValue > getJointLimitHigh())
        {
            res = false;
        }

        if (!res && verbose)
        {
            VR_INFO << "Joint: " << getName() << ":"
                    << " joint value (" << jointValue << ")"
                    << " is out of joint boundaries (lo:" << jointLimitLo << ", hi: " << jointLimitHi << ")" << endl;
        }

        return res;
    }

    void ModelJoint::respectJointLimits(float& jointValue) const
    {
        ReadLockPtr r = getModel()->getReadLock();
        if (jointValue < jointLimitLo)
        {
            jointValue = jointLimitLo;
        }

        if (jointValue > jointLimitHi)
        {
            jointValue = jointLimitHi;
        }
    }

    void ModelJoint::setJointLimits(float lo, float hi)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        jointLimitHi = hi;
        jointLimitLo = lo;
    }


    void ModelJoint::setLimitless(bool limitless)
    {
        this->limitless = limitless;
    }

    bool ModelJoint::isLimitless()
    {
        return limitless;
    }

    float ModelJoint::getDelta(float target)
    {
        float delta = 0.0f;

        // we check if the given target value violates our joint limits
        if (!limitless)
        {
            if (target < jointLimitLo || target > jointLimitHi)
            {
                return delta;
            }
        }

        delta = target - jointValue;

        // eventually take the other way around if it is shorter and if this joint is limitless.
        if (limitless && (std::abs(delta) > M_PI))
        {
            delta = float((-1.0f) * ((delta > 0) - (delta < 0)) * ((2 * M_PI) - std::abs(delta)));
        }

        return delta;
    }

    ModelJoint::ModelJoint() :
        ModelNode()
    {

    }

    float ModelJoint::getJointValueOffset() const
    {
        // never updated -> no lock needed
        return jointValueOffset;
    }


    float ModelJoint::getJointLimitHigh() const
    {
        ReadLockPtr lock = getModel()->getReadLock();
        return jointLimitHi;
    }


    float ModelJoint::getJointLimitLow() const
    {
        ReadLockPtr lock = getModel()->getReadLock();
        return jointLimitLo;
    }

    void ModelJoint::setMaxVelocity(float maxVel)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        maxVelocity = maxVel;
    }

    void ModelJoint::setMaxAcceleration(float maxAcc)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        maxAcceleration = maxAcc;
    }

    void ModelJoint::setMaxTorque(float maxTo)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        maxTorque = maxTo;
    }

    float ModelJoint::getMaxVelocity() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return maxVelocity;
    }

    float ModelJoint::getMaxAcceleration() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return maxAcceleration;
    }

    float ModelJoint::getMaxTorque() const
    {
        ReadLockPtr r = getModel()->getReadLock();
        return maxTorque;
    }

    void ModelJoint::propagateJointValue(const std::string& jointName, float factor)
    {
        WriteLockPtr w = getModel()->getWriteLock();
        if (factor == 0.0f)
        {
            propagatedJointValues.erase(jointName);
        }
        else
        {
            propagatedJointValues[jointName] = factor;
        }
    }

    void ModelJoint::updatePoseInternally(bool updateChildren, bool updateAttachments)
    {
        ModelPtr modelShared = getModel();

        for (auto it = propagatedJointValues.begin(); it != propagatedJointValues.end(); it++)
        {
            ModelNodePtr node = modelShared->getNode(it->first);

            if (!node || !checkNodeOfType(node, ModelNode::NodeType::Joint))
            {
                VR_WARNING << "Could not propagate joint value from " << getName()
                           << " to " << it->first << " because dependent joint does not exist..." << std::endl;
            }
            else
            {
                // TODO: check, if update is needed
                std::static_pointer_cast<ModelJoint>(node)->setJointValue(jointValue * it->second);
            }
        }
        ModelNode::updatePoseInternally(updateChildren, updateAttachments);
    }
}
