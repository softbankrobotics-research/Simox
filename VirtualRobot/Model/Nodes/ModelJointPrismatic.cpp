#include "ModelJointPrismatic.h"
#include "../Model.h"
#include "../../XML/BaseIO.h"
#include "Attachments/ModelNodeAttachment.h"

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
        result->setLimitless(limitless);
        return result;
    }

    std::string ModelJointPrismatic::toXML(const std::string &basePath, const std::string &modelPathRelative, bool storeAttachments)
    {
        std::stringstream ss;
        std::string pre = "\t";
        std::string pre2 = "\t\t";
        std::string pre3 = "\t\t\t";
        ss << pre << "<ModelNode name='" << name << "'>\n";
        if (!this->getStaticTransformation().isIdentity())
        {
            ss << pre2 << "<Transform>" << endl;
            ss << BaseIO::toXML(getStaticTransformation(), "\t\t\t");
            ss << pre2 << "</Transform>" << endl;
        }
        ss << pre2 << "<Joint type='prismatic'>\n";
        ss << pre3 << "<translationdirection x='" << getJointTranslationDirection()[0] << "' y='" << getJointTranslationDirection()[1] << "' z='" << getJointTranslationDirection()[2] << "'/>" << endl;
        ss << pre3 << "<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='mm'/>" << endl;
        ss << pre3 << "<MaxAcceleration value='" << maxAcceleration << "'/>" << endl;
        ss << pre3 << "<MaxVelocity value='" << maxVelocity << "'/>" << endl;
        ss << pre3 << "<MaxTorque value='" << maxTorque << "'/>" << endl;

        std::map< std::string, float >::iterator propIt = propagatedJointValues.begin();

        while (propIt != propagatedJointValues.end())
        {
            ss << pre3 << "<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << endl;
            propIt++;
        }

        ss << pre2 << "</Joint>\n";
        if (storeAttachments)
        {
            for (auto &a:getAttachments())
            {
                ss << a->toXML(basePath, modelPathRelative);
            }
        }

        std::vector<ModelNodePtr> children = this->getChildNodes();
        for (size_t i = 0; i < children.size(); i++)
        {
                ss << pre2 << "<Child name='" << children[i]->getName() << "'/>" << endl;
        }

        ss << pre << "</ModelNode>\n";
        return ss.str();
    }
}
