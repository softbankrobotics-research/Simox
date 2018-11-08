#include "MjcfDocument.h"

#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>

#include "utils.h"
#include "xml_visitors.h"


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

Element* Document::addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2)
{
    Element* texSkybox = addNewElement(asset(), "texture");
    
    texSkybox->SetAttribute("type", "skybox");
    texSkybox->SetAttribute("builtin", "gradient");
    texSkybox->SetAttribute("width", 128);
    texSkybox->SetAttribute("height", 128);
    texSkybox->SetAttribute("rgb1", toAttr(rgb1).c_str());
    texSkybox->SetAttribute("rgb2", toAttr(rgb2).c_str());   
    
    return texSkybox;
}

Element* Document::addNewElement(Element* parent, const std::string& elemName, bool first)
{
    Element* elem = NewElement(elemName.c_str());
    if (first)
    {
        parent->InsertFirstChild(elem);
    }
    else
    {
        parent->InsertEndChild(elem);
    }
    return elem;
}

Element* Document::addBodyElement(Element* parent, RobotNodePtr node)
{
    Element* body = addNewElement(parent, "body");
    body->SetAttribute("name", node->getName().c_str());

    if (node->hasParent())
    {
        Eigen::Matrix4f tf = node->getTransformationFrom(node->getParent());
        setBodyPose(body, tf);
    }
    
    if (node->isRotationalJoint() || node->isTranslationalJoint())
    {
        addJointElement(body, node);
    }
    
    addInertialElement(body, node);
    
    return body;
}


Element* Document::addGeomElement(Element* body, const std::string& meshName)
{
    assertElementIsBody(body);
    
    Element* geom = addNewElement(body, "geom", true);
    
    geom->SetAttribute("type", "mesh");
//    geom->SetAttribute("type", "capsule");
    geom->SetAttribute("mesh", meshName.c_str());
    //geom->SetAttribute("density", 100);
    
    return geom;
}

Element* Document::addInertialElement(Element* body, RobotNodePtr node)
{
    assertElementIsBody(body);
    
    Eigen::Matrix3f matrix = node->getInertiaMatrix();
    if (matrix.isIdentity(floatCompPrecision) && node->getMass() < floatCompPrecision)
    {
        // dont set an inertial element and let it be derived automatically
        return nullptr;
    }
    
    Element* inertial = addNewElement(body, "inertial");
    
    Eigen::Vector3f pos = node->getCoMLocal() * lengthScaling;
    inertial->SetAttribute("pos", toAttr(pos).c_str());
    inertial->SetAttribute("mass", double(node->getMass()));
    
    if (matrix.isDiagonal(floatCompPrecision))
    {
        Eigen::Vector3f diag = matrix.diagonal();
        inertial->SetAttribute("diaginertia", toAttr(diag).c_str());
    }
    else
    {
        /* from mujoco xml reference:
         * "Full inertia matrix M. Since M is 3-by-3 and symmetric, it is 
         * specified using only 6 numbers in the following order: 
         * M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)." */
        Eigen::Matrix<float, 6, 1> inertia;
        inertia << matrix(0, 0), matrix(1, 1), matrix(2, 2),
                matrix(0, 1), matrix(0, 2), matrix(1, 2);
        inertial->SetAttribute("fullinertia", toAttr(inertia).c_str());
    }
    
    return inertial;
}

Element* Document::addDummyInertial(Element* body)
{
    assertElementIsBody(body);
    
    Element* inertial = addNewElement(body, "inertial");
    
    inertial->SetAttribute("pos", toAttr(Eigen::Vector3f(0, 0, 0)).c_str());
    inertial->SetAttribute("mass", dummyMass);
    inertial->SetAttribute("diaginertia", toAttr(Eigen::Vector3f(1, 1, 1)).c_str());
    
    return inertial;
}


Element* Document::addJointElement(Element* body, RobotNodePtr node)
{
    assert(node->isRotationalJoint() xor node->isTranslationalJoint());
    
    Element* joint = addNewElement(body, "joint");
    
    joint->SetAttribute("name", node->getName().c_str());
    
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
    setJointAxis(joint, axis);
    joint->SetAttribute("limited", toAttr(!node->isLimitless()).c_str());
    
    if (!node->isLimitless())
    {
        Eigen::Vector2f range(node->getJointLimitLow(), node->getJointLimitHigh());
        if (node->isTranslationalJoint())
        {
            range *= lengthScaling;
        }
        joint->SetAttribute("range", toAttr(range).c_str());
    }
    
    return joint;
}

Element* Document::addMeshElement(const std::string& name, const std::string& file)
{
    Element* mesh = addNewElement(asset(), "mesh");
    
    mesh->SetAttribute("name", name.c_str());
    mesh->SetAttribute("file", file.c_str());
    
    return mesh;
}

void Document::setBodyPose(Element* body, const Eigen::Matrix4f& pose)
{
    Eigen::Vector3f pos = pose.block<3,1>(0, 3) * lengthScaling;
    Eigen::Matrix3f oriMat = pose.block<3,3>(0, 0);
    
    Eigen::Quaternionf quat(oriMat);
    
    if (!pos.isZero(floatCompPrecision))
    {
        body->SetAttribute("pos", toAttr(pos).c_str());
    }
    else
    {
        body->DeleteAttribute("pos");
    }
    
    if (!quat.isApprox(Eigen::Quaternionf::Identity(), floatCompPrecision))
    {
        body->SetAttribute("quat", toAttr(quat).c_str());
    }
    else
    {
        body->DeleteAttribute("quat");
    }
}

void Document::setJointAxis(Element* joint, const Eigen::Vector3f& axis)
{
    assertElementIs(joint, "joint");
    joint->SetAttribute("axis", toAttr(axis).c_str());
}


void Document::addContactExcludes(Element* rootBody)
{
    ContactExcludeVisitor visitor(*this);
    rootBody->Accept(&visitor);
}

Element* Document::addContactExclude(const Element& body1, const Element& body2)
{
    Element* exclude = addNewElement(contact(), "exclude");
    
    exclude->SetAttribute("body1", body1.Attribute("name"));
    exclude->SetAttribute("body2", body2.Attribute("name"));
    
    return exclude;
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




