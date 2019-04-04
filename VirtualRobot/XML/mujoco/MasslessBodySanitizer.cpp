#include "MasslessBodySanitizer.h"

#include <boost/algorithm/string/join.hpp>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/math/Helpers.h>


namespace VirtualRobot::mujoco
{

using namespace mjcf;

MasslessBodySanitizer::MasslessBodySanitizer(RobotPtr& robot) :
    robot(robot)
{}

void MasslessBodySanitizer::sanitize(Body root)
{
    // merge body leaf nodes with parent if they do not have a mass (inertial or geom)
    
    for (Body body = root.firstChild<Body>();
         body; body = body.nextSiblingElement<Body>())
    {
        sanitizeRecursion(body);
    }
}


void MasslessBodySanitizer::sanitizeRecursion(mjcf::Body body)
{
    RobotNodePtr bodyNode = robot->getRobotNode(body.name);
    Eigen::Matrix4f accChildPose = Eigen::Matrix4f::Identity();
    
    while (!body.hasMass())
    {
        std::cout << t << body.name << ": \t";
        
        if (!body.hasChild<Body>())
        {
            // leaf => end of recursion
            sanitizeLeafBody(body);
            return;
        }
        
        // non-leaf body
        // check whether there is only one child body
        Body childBody = body.firstChild<Body>();
        if (!childBody.nextSiblingElement<Body>())
        {
            mergeBodies(body, childBody, accChildPose);
        }
        else
        {
            std::cout << "Adding dummy inertial to massless body with >1 child bodies." << std::endl;
            // add a small mass
            body.addDummyInertial();
            break;
        }
    }
    
    // recursive descend
    for (Body child = body.firstChild<Body>();
         child; child = child.nextSiblingElement<Body>())
    {
        sanitizeRecursion(child);
    }
}

static void updatePos(AnyElement element, const Eigen::Matrix4f& accChildPose)
{
    const Eigen::Vector3f pos = element.getAttribute<Eigen::Vector3f>("pos", Eigen::Vector3f::Zero());
    element.setAttribute("pos", math::Helpers::TransformPosition(accChildPose, pos));
}

static void updateOri(AnyElement element, const Eigen::Matrix3f& accChildOri)
{
    if (element.is<Joint>())
    {
        Joint joint = element.as<Joint>();
        joint.axis = accChildOri * joint.axis.get();
    }
    else if (element.is<Light>())
    {
        Light light = element.as<Light>();
        light.dir = accChildOri * light.dir.get();
    }
    else
    {
        const Eigen::Quaternionf quat = 
                element.getAttribute<Eigen::Quaternionf>("quat", Eigen::Quaternionf::Identity());
        element.setAttribute("quat", Eigen::Quaternionf(accChildOri * quat));
    }
}

void copyChildren(Body body, Body child, const Eigen::Matrix4f& childPose)
{
    // merge childBody into body => move all its elements here
    // while doing this, apply accChildPose to elements
    for (AnyElement grandChild = child.firstChild<AnyElement>();
         grandChild; grandChild = grandChild.template nextSiblingElement<AnyElement>())
    {
        // clone grandchild
        AnyElement elem = grandChild.deepClone();
        
        if (elem)
        {
            /* Adapt pose/axis elements in child. Their poses/axes will be
             * relative to body's frame, so the transformation from body
             * to child will be lost. Therefore, apply accChildPose to
             * their poses/axes. */
            
            updatePos(elem, childPose);
            updateOri(elem, math::Helpers::Orientation(childPose));
        }
        
        // insert to body
        body.insertEndChild(elem);
    }
}



void MasslessBodySanitizer::mergeBodies(Body body, Body childBody, Eigen::Matrix4f& accChildPose)
{
    std::cout << "Merging with '" << childBody.name << "' " << std::endl;

    RobotNodePtr childNode = robot->getRobotNode(childBody.name);
    Eigen::Matrix4f childPose = childNode->getTransformationFrom(childNode->getParent());
    
    // update accumulated child pose
    // merged child's frame w.r.t. body's frame
    accChildPose = childPose * accChildPose;
    
    // merge childBody into body => move all its elements here
    // while doing this, apply accChildPose to elements
    copyChildren(body, childBody, accChildPose);
    
    // update body name
    MergedBodySet& bodySet = getMergedBodySetWith(body.name);
    bodySet.addBody(childBody.name);
    body.name = bodySet.getMergedBodyName();
    
    std::cout << t << "\t(new name: '" << bodySet.getMergedBodyName() << "')" << std::endl;
    
    // delete child
    body.deleteChild(childBody);
}


void MasslessBodySanitizer::sanitizeLeafBody(Body body)
{
    VR_ASSERT_MESSAGE(!body.hasChild<Body>(), "Leaf body must not have a child body.");
    VR_ASSERT_MESSAGE(!body.hasMass(), "Leaf body must not have mass.");
    
    if (!body.hasChildren()) // is completely empty?
    {
        // leaf without geom: make it a site
        std::cout << "Changing to site." << std::endl;
        body.transform<Site>();
    }
    else
    {
        // add a small mass
        std::cout << "Adding dummy inertial to massless leaf body with children." << std::endl;
        body.addDummyInertial();
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
= default;

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

}
