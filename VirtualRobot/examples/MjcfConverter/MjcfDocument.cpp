#include "MjcfDocument.h"

#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>


using namespace VirtualRobot;
using namespace mjcf;


Document::Document() : root(NewElement("mujoco"))
{
    // create root element
    this->InsertEndChild(root);
}

void Document::setModelName(const std::string& name)
{
    root->SetAttribute("model", name.c_str());
}

Element* Document::addNewElement(Element* parent, const std::string& elemName)
{
    Element* elem = NewElement(elemName.c_str());
    parent->InsertEndChild(elem);
    return elem;
}

Element*Document::addBodyElement(Element* parent, RobotNodePtr node)
{
    Element* body = addNewElement(parent, "body");
    body->SetAttribute("name", node->getName().c_str());

    if (node->hasParent())
    {
        Eigen::Matrix4f tf = node->getTransformationFrom(node->getParent());
        Eigen::Vector3f pos = tf.block<3,1>(0, 3);
        Eigen::Matrix3f oriMat = tf.block<3,3>(0, 0);
        
        Eigen::Quaternionf quat(oriMat);
    
        if (!pos.isZero(floatCompPrecision))
        {
            body->SetAttribute("pos", toAttr(pos).c_str());
        }
        if (!quat.isApprox(Eigen::Quaternionf::Identity(), floatCompPrecision))
        {
            body->SetAttribute("quat", toAttr(quat).c_str());
        }
    }
    
    if (node->isRotationalJoint() || node->isTranslationalJoint())
    {
        addJointelement(body, node);
    }
    node->getOptionalDHParameters();
    return body;
}

Element* Document::addJointelement(Element* body, RobotNodePtr node)
{
    assert(node->isRotationalJoint() xor node->isTranslationalJoint());
    
    Element* joint = addNewElement(body, "joint");
    
    {
        std::stringstream jointName;
        jointName << node->getName() << "_joint";
        joint->SetAttribute("name", jointName.str().c_str());
    }
    
    // get the axis
    Eigen::Vector3f axis;
    if (node->isRotationalJoint())
    {
        RobotNodeRevolutePtr revolute = boost::dynamic_pointer_cast<RobotNodeRevolute>(node);
        assert(revolute);
        axis = revolute->getJointRotationAxisInJointCoordSystem();
    }
    else if (node->isTranslationalJoint())
    {
        RobotNodePrismaticPtr prismatic = boost::dynamic_pointer_cast<RobotNodePrismatic>(node);
        assert(prismatic);
        axis = prismatic->getJointTranslationDirectionJointCoordSystem();
    }
    
    joint->SetAttribute("type", node->isRotationalJoint() ? "hinge" : "slide");
    joint->SetAttribute("axis", toAttr(axis).c_str());
    joint->SetAttribute("limited", toAttr(!node->isLimitless()).c_str());
    
    if (!node->isLimitless())
    {
        Eigen::Vector2f range(node->getJointLimitLow(), node->getJointLimitHigh());
        joint->SetAttribute("range", toAttr(range).c_str());
    }
    
    return joint;
}

std::string Document::toAttr(bool b)
{
    static const std::string strings[] = { "false", "true" };
    return strings[int(b)];
}

std::string Document::toAttr(const Eigen::Quaternionf& quat)
{
    std::stringstream ss;
    ss << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
    return ss.str();
}

Element* Document::topLevelElement(const std::string& name)
{
    Element* elem = root->FirstChildElement(name.c_str());
    if (!elem)
    {
        elem = addNewElement(root, name);
    }
    return elem;
}


