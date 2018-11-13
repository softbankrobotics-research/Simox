#include "MjcfMasslessBodySanitizer.h"

#include <boost/algorithm/string/join.hpp>


#include "utils.h"


using namespace VirtualRobot;
using namespace mjcf;
namespace tx = tinyxml2;


MjcfMasslessBodySanitizer::MjcfMasslessBodySanitizer(DocumentPtr& document, RobotPtr& robot) :
    document(document), robot(robot)
{
    
}

void MjcfMasslessBodySanitizer::sanitize()
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


void MjcfMasslessBodySanitizer::sanitizeRecursion(Element* body)
{
    assertElementIsBody(body);

    std::cout << "- Node '" << body->Attribute("name") << "': " << std::endl;
    const std::string t = "  | ";
    
    std::string bodyName = body->Attribute("name");
    RobotNodePtr bodyNode = robot->getRobotNode(bodyName);
    Eigen::Matrix4f bodyPose = bodyNode->getTransformationFrom(bodyNode->getParent());
    
    while (!hasMass(body))
    {
        std::cout << t << "Massless" << std::endl;
        
        if (!hasElementChild(body, "body"))
        {
            // leaf => end of recursion
            std::cout << t << "Leaf" << std::endl;
            sanitizeLeafBody(body);
            return; 
        }
        
        // non-leaf body
        std::cout << t << "Non-leaf" << std::endl;
        
        // check whether there is only one child body
        Element* childBody = body->FirstChildElement("body");
        if (!childBody->NextSiblingElement("body"))
        {
            std::string childBodyName = childBody->Attribute("name");
            
            std::cout << t << "Single child body => merging '" << childBodyName
                      << "' into '" << bodyName << "'" << std::endl;

            // check child for pose attributes
            if (childBody->Attribute("pos") || childBody->Attribute("quat"))
            {
                // update body's transform and joint axes
                
                RobotNodePtr childNode = robot->getRobotNode(childBodyName);
                Eigen::Matrix4f childPose = childNode->getTransformationFrom(childNode->getParent());
                
                // body's pose
                bodyPose = bodyPose * childPose;
                document->setBodyPose(body, bodyPose);
                
                /* Adapt axes of joints in body:
                 * The axes will be relative to the new pose. Therefore, the additional rotation
                 * of the child must be subtracted from the joint axis.
                 */
                
                Eigen::Matrix3f revChildOri = childPose.block<3,3>(0, 0).transpose();
                
                for (Element* joint = body->FirstChildElement("joint"); joint;
                     joint = joint->NextSiblingElement("joint"))
                {
                    Eigen::Vector3f axis = strToVec(joint->Attribute("axis"));
                    // apply child orientation
                    axis = revChildOri * axis;
                    document->setJointAxis(joint, axis);
                }
            }
            
            
            // merge childBody into body => move all its elements here
            for (tx::XMLNode* grandChild = childBody->FirstChild(); grandChild;
                 grandChild = grandChild->NextSibling())
            {
                std::cout << t << " | Moving '" << grandChild->Value() << "'" << std::endl;
                
                // clone grandchild
                tx::XMLNode* grandChildClone = grandChild->DeepClone(nullptr);
                
                // insert to body
                body->InsertEndChild(grandChildClone);
            }
            
            // update body name
            MergedBodySet& bodySet = getMergedBodySetWith(bodyName);
            bodySet.addBody(childBodyName);
            body->SetAttribute("name", bodySet.getMergedBodyName().c_str());
            
            // delete child
            body->DeleteChild(childBody);
            
            std::cout << t << "=> New body name: " << bodySet.getMergedBodyName() << std::endl;
        }
        else
        {
            std::cout << "[WARN]" << t << "Massless body with >1 child body: " 
                       << body->Attribute("name") << std::endl;
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

void MjcfMasslessBodySanitizer::sanitizeLeafBody(Element* body)
{
    assert(!hasElementChild(body, "body"));
    assert(!hasMass(body));
    
    if (body->NoChildren()) // is completely empty?
    {
        // leaf without geom: make it a site
        std::cout << "  | Empty => Changing body '" << body->Attribute("name") << "' to site." << std::endl;
        body->SetValue("site");
    }
    else
    {
        // add a small mass
        std::cout << "  | Not empty => Adding dummy inertial." << std::endl;
        document->addDummyInertial(body);
    }
}

const std::vector<MergedBodySet>& MjcfMasslessBodySanitizer::getMergedBodySets() const
{
    return mergedBodySets;
}

const std::string& MjcfMasslessBodySanitizer::getMergedBodyName(const std::string& originalBodyName)
{
    return getMergedBodySetWith(originalBodyName).getMergedBodyName();
}

MergedBodySet&MjcfMasslessBodySanitizer::getMergedBodySetWith(const std::string& bodyName)
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
    originalBodyNames.insert(bodyName);
    updateMergedBodyName();
}

bool MergedBodySet::containsBody(const std::string& bodyName) const
{
    return originalBodyNames.find(bodyName) != originalBodyNames.end();
}

const std::string& MergedBodySet::getMergedBodyName() const
{
    return mergedBodyName;
}

void MergedBodySet::updateMergedBodyName()
{
    mergedBodyName = boost::algorithm::join(originalBodyNames, "~");
}
