#include "MjcfDocument.h"

#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>


using namespace VirtualRobot;
using namespace mjcf;


Document::Document()
{
    // create root element
    root = NewElement("mujoco");
    this->InsertEndChild(root);
}

void Document::setModelName(const std::string& name)
{
    root->SetAttribute("model", name.c_str());
}

void Document::setNewElementClass(const std::string& className, bool excludeBody)
{
    this->newElementClass = className;
    this->newElementClassExcludeBody = excludeBody;
}

XMLElement* Document::addSkyboxTexture(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2)
{
    XMLElement* texSkybox = addNewElement(asset(), "texture");
    
    texSkybox->SetAttribute("type", "skybox");
    texSkybox->SetAttribute("builtin", "gradient");
    texSkybox->SetAttribute("width", 128);
    texSkybox->SetAttribute("height", 128);
    texSkybox->SetAttribute("rgb1", toAttr(rgb1).c_str());
    texSkybox->SetAttribute("rgb2", toAttr(rgb2).c_str());   
    
    return texSkybox;
}

XMLElement* Document::addMocapBody(const std::string& name, float geomSize)
{
    /*
        <body name="hand-ctrl" mocap="true" >
            <geom type="box" size="0.01 0.01 0.01" contype="0" conaffinity="0" rgba="0.9 0.5 0.5 0.5" />
        </body>
      */
    XMLElement* mocap = addNewElement(worldbody(), "body");
    mocap->SetAttribute("name", name.c_str());
    mocap->SetAttribute("mocap", "true");
    
    // add geom for visualization
    
    XMLElement* geom = addNewElement(mocap, "geom");
    geom->SetAttribute("type", "box");
    setAttr(geom, "size", Eigen::Vector3f{geomSize, geomSize, geomSize});
    
    return mocap;
}



XMLElement* Document::addDefaultsClass(const std::string& className)
{
    XMLElement* def = addNewElement(default_(), "default", className);
    
    return def;
}

XMLElement* Document::addRobotRootBodyElement(const std::string& robotName)
{
    this->robotRootBody_ = addNewElement(worldbody(), "body");
    
    robotRootBody_->SetAttribute("name", robotName.c_str());
    robotRootBody_->SetAttribute("childclass", robotName.c_str());
    
    return robotRootBody_;
}

XMLElement* Document::addBodyElement(XMLElement* parent, RobotNodePtr node)
{
    XMLElement* body = addNewElement(parent, "body");
    body->SetAttribute("name", node->getName().c_str());

    if (node->hasParent())
    {
        Eigen::Matrix4f tf = node->getTransformationFrom(node->getParent());
        setElementPose(body, tf);
    }
    
    if (node->isRotationalJoint() || node->isTranslationalJoint())
    {
        addJointElement(body, node);
    }
    
    addInertialElement(body, node);
    
    return body;
}


XMLElement* Document::addGeomElement(XMLElement* body, const std::string& meshName)
{
    assertElementIsBody(body);
    
    XMLElement* geom = addNewElement(body, "geom", "", true);
    
    geom->SetAttribute("type", "mesh");
    geom->SetAttribute("mesh", meshName.c_str());
    
    return geom;
}

XMLElement* Document::addInertialElement(XMLElement* body, RobotNodePtr node)
{
    assertElementIsBody(body);
    
    Eigen::Matrix3f matrix = node->getInertiaMatrix();
    if (matrix.isIdentity(floatCompPrecision) && node->getMass() < floatCompPrecision)
    {
        // dont set an inertial element and let it be derived automatically
        return nullptr;
    }
    
    XMLElement* inertial = addNewElement(body, "inertial");
    
    Eigen::Vector3f pos = node->getCoMLocal();
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

XMLElement*Document::addDummyInertial(XMLElement* body, bool first)
{
    assertElementIsBody(body);
    
    XMLElement* inertial = addNewElement(body, "inertial", "", first);
    
    inertial->SetAttribute("pos", toAttr(Eigen::Vector3f(0, 0, 0)).c_str());
    inertial->SetAttribute("mass", dummyMass);
    inertial->SetAttribute("diaginertia", toAttr(Eigen::Vector3f(1, 1, 1)).c_str());
    
    return inertial;
}


XMLElement* Document::addJointElement(XMLElement* body, RobotNodePtr node)
{
    assert(node->isRotationalJoint() xor node->isTranslationalJoint());
    
    XMLElement* joint = addNewElement(body, "joint");
    
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
        joint->SetAttribute("range", toAttr(range).c_str());
    }
    
    return joint;
}

XMLElement* Document::addFreeJointElement(XMLElement* body)
{
    assertElementIsBody(body);
    return addNewElement(body, "freejoint");
}

XMLElement* Document::addMeshElement(const std::string& name, const std::string& file, const std::string& className)
{
    XMLElement* mesh = addNewElement(asset(), "mesh", className);
    
    mesh->SetAttribute("name", name.c_str());
    mesh->SetAttribute("file", file.c_str());
    
    return mesh;
}

XMLElement* Document::addActuatorMotorElement(const std::string& jointName)
{
    return addActuatorShortcut("motor", jointName, jointName);
}

XMLElement*Document::addActuatorPositionElement(const std::string& jointName, float kp)
{
    return addActuatorShortcut("position", jointName, jointName, "kp", kp);
}

XMLElement*Document::addActuatorPositionElement(const std::string& jointName, bool ctrlLimited, 
                                                const Eigen::Vector2f& ctrlRange, float kp)
{
    XMLElement* act = addActuatorPositionElement(jointName, kp);
    act->SetAttribute("ctrllimited", toAttr(ctrlLimited).c_str());
    act->SetAttribute("ctrlrange", toAttr(ctrlRange).c_str());
    return act;
}

XMLElement*Document::addActuatorVelocityElement(const std::string& jointName, float kv)
{
    return addActuatorShortcut("velocity", jointName, jointName, "kv", kv);
}

XMLElement* Document::addEqualityWeld(const std::string& name, const std::string& body1, const std::string& body2, const std::string& className)
{
    XMLElement* weld = addNewElement(equality(), "weld", className);
    
    weld->SetAttribute("name", name.c_str());
    weld->SetAttribute("body1", body1.c_str());
    weld->SetAttribute("body2", body2.c_str());
    
    return weld;
}


XMLElement* Document::addActuatorShortcut(
        const std::string& type, const std::string& name, const std::string& jointName, 
        const std::string& paramName, float paramValue)
{
    XMLElement* motor = addNewElement(actuator(), type.c_str());
    
    motor->SetAttribute("name",  name.c_str());
    motor->SetAttribute("joint", jointName.c_str());
    
    if (!paramName.empty() && paramValue > floatCompPrecision)
    {
        motor->SetAttribute(paramName.c_str(), paramValue);
    }
    
    return motor;
}


void Document::setElementPose(XMLElement* elem, const Eigen::Matrix4f& pose)
{
    Eigen::Vector3f pos = pose.block<3,1>(0, 3);
    Eigen::Matrix3f oriMat = pose.block<3,3>(0, 0);
    
    Eigen::Quaternionf quat(oriMat);
    
    setElementPos(elem, pos);
    setElementQuat(elem, quat);
}

void Document::setElementPos(XMLElement* elem, Eigen::Vector3f pos)
{
    if (!pos.isZero(floatCompPrecision))
    {
        elem->SetAttribute("pos", toAttr(pos).c_str());
    }
    else
    {
        elem->DeleteAttribute("pos");
    }
}

void Document::setElementQuat(XMLElement* elem, const Eigen::Quaternionf& quat)
{
    if (!quat.isApprox(Eigen::Quaternionf::Identity(), floatCompPrecision))
    {
        elem->SetAttribute("quat", toAttr(quat).c_str());
    }
    else
    {
        elem->DeleteAttribute("quat");
    }
}

void Document::setJointAxis(XMLElement* joint, const Eigen::Vector3f& axis)
{
    assertElementIs(joint, "joint");
    joint->SetAttribute("axis", toAttr(axis).c_str());
}


XMLElement* Document::addContactExclude(const XMLElement& body1, const XMLElement& body2)
{
    return addContactExclude(body1.Attribute("name"), body2.Attribute("name"));
}

XMLElement* Document::addContactExclude(const std::string& body1Name, const std::string& body2Name)
{
    XMLElement* exclude = addNewElement(contact(), "exclude");
    
    exclude->SetAttribute("body1", body1Name.c_str());
    exclude->SetAttribute("body2", body2Name.c_str());
    
    return exclude;
}


XMLElement* Document::addNewElement(XMLElement* parent, const std::string& elemName, 
                                 const std::string& className, bool first)
{
    XMLElement* elem = NewElement(elemName.c_str());
    
    if (!className.empty())
    {
        elem->SetAttribute("class", className.c_str());
    }
    else if (!newElementClass.empty()
             && !(newElementClassExcludeBody && (isElement(parent, "body") || isElement(elem, "body")))
             && allowsClassAttr({parent->Value(), elemName}))
    {
        elem->SetAttribute("class", newElementClass.c_str());
    }
    
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

XMLElement* Document::getOrCreateElement(XMLElement* parent, const std::string& elemName)
{
    XMLElement* elem = parent->FirstChildElement(elemName.c_str());
    if (!elem)
    {
        elem = addNewElement(parent, elemName);
    }
    return elem;
}

XMLElement* Document::section(const std::string& name)
{
    return getOrCreateElement(root, name);
}

void Document::setDummyMass(float value)
{
    dummyMass = value;
}

void Document::setFloatCompPrecision(float value)
{
    floatCompPrecision = value;
}


XMLElement* Document::robotRootBody() const
{
    return robotRootBody_;
}


const std::set<Document::ElementType> Document::ELEM_NAMES_WITH_ATTR_CLASS = 
{
    {"asset", "mesh"    },
    {"asset", "material"},
    
    {"body", "joint"    },
    {"body", "geom"     },
    {"body", "site"     },
    {"body", "camera"   },
    {"body", "light"    },
    
    {"contact", "pair"  },
    
    {"equality", "connect"  },
    {"equality", "weld"     },
    {"equality", "joint"    },
    {"equality", "tendon"   },
    {"equality", "distance" },
    
    {"tendon", "spatial" },
    {"tendon", "fixed"   },
    
    {"actuator", "general"  },
    {"actuator", "motor"    },
    {"actuator", "position" },
    {"actuator", "velocity" },
    {"actuator", "cylinder" },
    {"actuator", "muscle"   },
};

bool Document::allowsClassAttr(const Document::ElementType& type)
{
    return ELEM_NAMES_WITH_ATTR_CLASS.find(type) != ELEM_NAMES_WITH_ATTR_CLASS.end();
}


