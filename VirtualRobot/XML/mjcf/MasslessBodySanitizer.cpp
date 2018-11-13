#include "MasslessBodySanitizer.h"
#include "utils.h"

#include <boost/algorithm/string/join.hpp>


using namespace VirtualRobot;
using namespace mjcf;
namespace tx = tinyxml2;


MasslessBodySanitizer::MasslessBodySanitizer(DocumentPtr& document, RobotPtr& robot) :
    document(document), robot(robot)
{
    
}

void MasslessBodySanitizer::sanitize()
{
    // merge body leaf nodes with parent if they do not have a mass (inertial or geom)
    
    Element* root = document->worldbody()->FirstChildElement("body");
    
    for (Element* body = root->FirstChildElement("body");
         body;
         body = body->NextSiblingElement("body"))
    {
        sanitizeRecursion(body);
    }
}


void MasslessBodySanitizer::sanitizeRecursion(Element* body)
{
    assertElementIsBody(body);

    std::string bodyName = body->Attribute("name");
    RobotNodePtr bodyNode = robot->getRobotNode(bodyName);
    Eigen::Matrix4f accChildPose = Eigen::Matrix4f::Identity();
    
    while (!hasMass(body))
    {
        std::cout << t << bodyName << ": \t";
        
        if (!hasElementChild(body, "body"))
        {
            // leaf => end of recursion
            sanitizeLeafBody(body);
            return;
        }
        
        // non-leaf body
        // check whether there is only one child body
        Element* childBody = body->FirstChildElement("body");
        if (!childBody->NextSiblingElement("body"))
        {
            mergeBodies(body, childBody, accChildPose);
        }
        else
        {
            std::cout << "[WARN] Massless body with >1 child bodies." << std::endl;
            break;
        }
    }
    
    
    // recursive descend
    
    for (Element* child = body->FirstChildElement("body");
         child;
         child = child->NextSiblingElement("body"))
    {
        sanitizeRecursion(child);
    }
    
}

void MasslessBodySanitizer::mergeBodies(Element* body, Element* childBody, 
                                            Eigen::Matrix4f& accChildPose)
{
    std::string childBodyName = childBody->Attribute("name");
    
    std::cout << "Merging with '" << childBodyName << "' ";

    RobotNodePtr childNode = robot->getRobotNode(childBodyName);
    Eigen::Matrix4f childPose = childNode->getTransformationFrom(childNode->getParent());
    
    // update accumulated child pose
    // merged child's frame w.r.t. body's frame
    accChildPose = childPose * accChildPose;
    Eigen::Matrix3f accChildOri = accChildPose.block<3,3>(0, 0);
    
    // merge childBody into body => move all its elements here
    // while doing this, apply accChildPose to elements
    for (tx::XMLNode* grandChild = childBody->FirstChild(); grandChild;
         grandChild = grandChild->NextSibling())
    {
        // clone grandchild
        tx::XMLNode* grandChildClone = grandChild->DeepClone(nullptr);
        
        Element* elem = grandChildClone->ToElement();
        if (elem)
        {
            /* Adapt pose/axis elements in child. Their poses/axes will be
                 * relative to body's frame, so the transformation from body
                 * to child will be lost. Therefore, apply accChildPose to
                 * their poses/axes. */
            
            if (isElement(elem, "joint"))
            {
                // update pos and axis
                updateChildPos(elem, accChildPose);
                updateChildAxis(elem, accChildOri);
            }
            else if (isElement(elem, "body") 
                     || isElement(elem, "inertial")
                     || isElement(elem, "geom") 
                     || isElement(elem, "site")
                     || isElement(elem, "camera"))
            {
                updateChildPos(elem, accChildPose);
                updateChildQuat(elem, accChildOri);
            }
            else if (isElement(elem, "light"))
            {
                updateChildPos(elem, accChildPose);
                updateChildAxis(elem, accChildOri, "dir");
            }
        }
        
        // insert to body
        body->InsertEndChild(grandChildClone);
    }
    
    // update body name
    MergedBodySet& bodySet = getMergedBodySetWith(body->Attribute("name"));
    bodySet.addBody(childBodyName);
    body->SetAttribute("name", bodySet.getMergedBodyName().c_str());
    
    std::cout << "(new name: " << bodySet.getMergedBodyName() << ")" << std::endl;
    
    // delete child
    body->DeleteChild(childBody);
}

void MasslessBodySanitizer::updateChildPos(Element* elem, const Eigen::Matrix4f& accChildPose)
{
    const char* posStr = elem->Attribute("pos");
    Eigen::Vector3f pos = posStr ? strToVec(posStr) : 
                                   Eigen::Vector3f::Zero();
    
    Eigen::Vector4f posHom;
    posHom << pos, 1;
    posHom = accChildPose * posHom;
    pos = posHom.head<3>();
    
    document->setElementPos(elem, pos);
}

void MasslessBodySanitizer::updateChildQuat(Element* elem, const Eigen::Matrix3f& accChildOri)
{
    const char* quatStr = elem->Attribute("quat");
    Eigen::Quaternionf quat = quatStr ? strToQuat(quatStr) : 
                                        Eigen::Quaternionf::Identity();
    
    quat = accChildOri * quat;
    document->setElementQuat(elem, quat);
}

void MasslessBodySanitizer::updateChildAxis(Element* elem, const Eigen::Matrix3f& accChildOri, 
                                                const char* attrName)
{
    Eigen::Vector3f axis = strToVec(elem->Attribute(attrName));
    axis = accChildOri * axis;
    elem->SetAttribute(attrName, toAttr(axis).c_str());
    
    if (strcmp(attrName, "axis") == 0)
    {
        document->setJointAxis(elem, axis);
    }
    else
    {
        elem->SetAttribute(attrName, toAttr(axis).c_str());
    }
}

void MasslessBodySanitizer::sanitizeLeafBody(Element* body)
{
    assert(!hasElementChild(body, "body"));
    assert(!hasMass(body));
    
    if (body->NoChildren()) // is completely empty?
    {
        // leaf without geom: make it a site
        std::cout << "Changing to site." << std::endl;
        body->SetValue("site");
    }
    else
    {
        // add a small mass
        std::cout << "Adding dummy inertial." << std::endl;
        document->addDummyInertial(body);
    }
}

const std::vector<MergedBodySet>& MasslessBodySanitizer::getMergedBodySets() const
{
    return mergedBodySets;
}

const std::string& MasslessBodySanitizer::getMergedBodyName(const std::string& originalBodyName)
{
    return getMergedBodySetWith(originalBodyName).getMergedBodyName();
}

MergedBodySet&MasslessBodySanitizer::getMergedBodySetWith(const std::string& bodyName)
{
    for (auto& set : mergedBodySets)
    {
        if (set.containsBody(bodyName))
        {
           return set;
        }
    }
    
    // not found => add
    mergedBodySets.push_back(MergedBodySet(bodyName));
    
    return mergedBodySets.back();
}


MergedBodySet::MergedBodySet()
{
}

MergedBodySet::MergedBodySet(const std::string& bodyName)
{
    addBody(bodyName);
}

void MergedBodySet::addBody(const std::string& bodyName)
{
    originalBodyNames.push_back(bodyName);
    updateMergedBodyName();
}

bool MergedBodySet::containsBody(const std::string& bodyName) const
{
    return std::find(originalBodyNames.begin(), originalBodyNames.end(),
                     bodyName) != originalBodyNames.end();
}

const std::string& MergedBodySet::getMergedBodyName() const
{
    return mergedBodyName;
}

void MergedBodySet::updateMergedBodyName()
{
    mergedBodyName = boost::algorithm::join(originalBodyNames, "~");
}
