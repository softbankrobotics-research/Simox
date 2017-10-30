#include "ModelJointRevolute.h"
#include "../../XML/BaseIO.h"
#include "Attachments/ModelNodeAttachment.h"

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

    std::string ModelJointRevolute::toXML(const std::string &basePath, const std::string &modelPathRelative, bool storeAttachments)
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
        ss << pre2 << "<Joint type='revolute'>\n";
        ss << pre3 << "<axis x='" << this->axis[0] << "' y='" << axis[1] << "' z='" << axis[2] << "'/>" << endl;
        ss << pre3 << "<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='radian'/>" << endl;
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
