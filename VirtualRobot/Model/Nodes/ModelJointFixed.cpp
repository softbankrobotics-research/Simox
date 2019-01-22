#include "ModelJointFixed.h"
#include "../../XML/BaseIO.h"
#include "Attachments/ModelNodeAttachment.h"

namespace VirtualRobot
{
    ModelJointFixed::ModelJointFixed(const ModelWeakPtr& model, const std::string& name, const Eigen::Matrix4f& localTransformation)
            : ModelJoint(model, name, localTransformation, 0, 0, 0.0f)
    {
    }

    ModelJointFixed::~ModelJointFixed()
    {
    }

    ModelNode::NodeType ModelJointFixed::getType() const
    {
        return ModelNode::NodeType::JointFixed;
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

    std::string ModelJointFixed::toXML(const std::string &basePath, const std::string &modelPathRelative, bool storeAttachments)
    {
        std::stringstream ss;
        std::string pre = "\t";
        std::string pre2 = "\t\t";
        ss << pre << "<ModelNode name='" << getName() << "'>\n";
        if (!this->getLocalTransformation().isIdentity())
        {
            ss << pre2 << "<Transform>" << endl;
            ss << BaseIO::toXML(getLocalTransformation(), "\t\t\t");
            ss << pre2 << "</Transform>" << endl;
        }
        ss << pre2 << "<Joint type='fixed'/>\n";
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



    ModelNodePtr ModelJointFixed::_clone(ModelPtr newModel, float scaling)
    {
        ModelJointFixedPtr result(new ModelJointFixed(newModel, getName(), getLocalTransformation()));
        return result;
    }
}
