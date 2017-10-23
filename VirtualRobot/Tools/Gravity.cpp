#include "Gravity.h"


#include "../Model/Nodes/ModelJoint.h"
#include "../Model/Nodes/ModelJointPrismatic.h"
#include "../Model/Nodes/ModelJointRevolute.h"
#include "../Model/Nodes/ModelLink.h"
#include "../Model/JointSet.h"
#include "../Model/LinkSet.h"
#include "../Model/Model.h"
#include "../VirtualRobot.h"
#include "../VirtualRobotException.h"
#include "../Tools/MathTools.h"


using namespace VirtualRobot;

Gravity::Gravity(VirtualRobot::ModelPtr robot, VirtualRobot::JointSetPtr rnsJoints, VirtualRobot::LinkSetPtr rnsBodies) :
    robot(robot),
    rns(rnsJoints),
    rnsBodies(rnsBodies)
{
    THROW_VR_EXCEPTION_IF(!robot || !rns || !rnsBodies || rns->getSize()==0, "NULL data");


    nodes = rns->getJoints();
    nodesBodies = rnsBodies->getLinks();

    for (auto& node : nodes)
    {
        if (!node->isRotationalJoint())
        {
            THROW_VR_EXCEPTION("Not a rotational joint:" << node->getName());
        }
    }

    for (auto& body : nodesBodies)
    {
        if (body->getMass() <= 0)
        {
            THROW_VR_EXCEPTION("No mass for body" << body->getName());
        }
    }

    gravityDataHelperRoot = GravityData::create(robot->getRootNode(), nodes, nodesBodies, gravityDataHelperVec);

    children.resize(nodes.size(), nodesBodies.size());
    // build children map
    unsigned int a = 0;
    unsigned int b = 0;
    for (auto& node : nodes)
    {
        b=0;
        for (auto& body : nodesBodies)
        {
            if (node->hasChild(body, true) /*|| node == body*/)
            {
                children(a,b) = 1;
            } else
                children(a,b) = 0;
            b++;
        }
        a++;
    }

}

Gravity::~Gravity()
{
}

void Gravity::computeGravityTorque(std::vector<float> &storeValues)
{
    storeValues.resize(rns->getSize());

    unsigned int pos = 0;
    unsigned int a = 0;
    unsigned int b = 0;
    for (auto& node : nodes)
    {
        VirtualRobot::ModelJointRevolutePtr rnRevolute = std::dynamic_pointer_cast<VirtualRobot::ModelJointRevolute>(node);
        Eigen::Vector3f axisGlobal = rnRevolute->getJointRotationAxis();
        Eigen::Vector3f jointPosGlobal = rnRevolute->getGlobalPose().block(0, 3, 3, 1);
        axisGlobal.normalize();

        float gravityJoint = 0;

        b = 0;
        for (auto& body : nodesBodies)
        {
            if (children(a,b) == 1) //node->hasChild(body, true))
            {
                Eigen::Vector3f comGlobal = body->getCoMGlobal();
                VirtualRobot::MathTools::BaseLine<Eigen::Vector3f> l(jointPosGlobal, axisGlobal);
                Eigen::Vector3f pointOnAxis = VirtualRobot::MathTools::nearestPointOnLine<Eigen::Vector3f>(l, comGlobal);
                Eigen::Vector3f r = comGlobal - pointOnAxis; // vector from axis to com (change sign?)
                r *= 0.001f; // mm -> m
                Eigen::Vector3f F(0, 0, -9.81f);
                F *= body->getMass();
                gravityJoint += (r.cross(F)).dot(axisGlobal);
            }
            b++;
        }

        // gravity compensation: invert the gravity torque
        gravityJoint *= -1.0f;
        storeValues[pos] = gravityJoint;
        pos++;
        a++;
    }
}

void Gravity::computeGravityTorqueOptimized(std::vector<float> &storeValues)
{
    storeValues.resize(gravityDataHelperVec.size());
    Eigen::Vector3f comGlobal;
    gravityDataHelperRoot->computeCoMAndTorque(comGlobal);
    for (size_t i = 0; i < gravityDataHelperVec.size(); ++i)
    {
        storeValues.at(i) = gravityDataHelperVec[i]->torque;
    }
}

std::map<std::string, float> Gravity::computeGravityTorque()
{
    std::vector<float> storeValues;
    storeValues.resize(nodes.size());
    computeGravityTorque(storeValues);
    std::map<std::string, float> torques;
    unsigned int a = 0;
    for (auto& node : nodes)
    {
        torques[node->getName()] = storeValues.at(a);
        a++;
    }
    /*
    std::map<std::string, float> torques;

    std::vector<VirtualRobot::RobotNodePtr> nodes = rns->getAllRobotNodes();
    std::vector<VirtualRobot::RobotNodePtr> nodesBodies = rnsBodies->getAllRobotNodes();

    for (auto& node : nodes)
    {
        VirtualRobot::RobotNodeRevolutePtr rnRevolute = boost::dynamic_pointer_cast<VirtualRobot::RobotNodeRevolute>(node);
        Eigen::Vector3f axisGlobal = rnRevolute->getJointRotationAxis();
        Eigen::Vector3f jointPosGlobal = rnRevolute->getGlobalPose().block(0, 3, 3, 1);
        axisGlobal.normalize();

        float gravityJoint = 0;

        for (auto& body : nodesBodies)
        {
            if (node->hasChild(body, true))
            {
                Eigen::Vector3f comGlobal = body->getCoMGlobal();
                VirtualRobot::MathTools::BaseLine<Eigen::Vector3f> l(jointPosGlobal, axisGlobal);
                Eigen::Vector3f pointOnAxis = VirtualRobot::MathTools::nearestPointOnLine<Eigen::Vector3f>(l, comGlobal);
                Eigen::Vector3f r = comGlobal - pointOnAxis; // vector from axis to com (change sign?)
                r *= 0.001f; // mm -> m
                Eigen::Vector3f F(0, 0, -9.81);
                F *= body->getMass();
                gravityJoint += (r.cross(F)).dot(axisGlobal);
            }
        }

        // gravity compensation: invert the gravity torque
        gravityJoint *= -1.0f;
        torques[node->getName()] = gravityJoint;
    }*/
    return torques;
}

Gravity::GravityData::GravityData()
{




}

Gravity::GravityDataPtr Gravity::GravityData::create(ModelNodePtr node, const std::vector<ModelJointPtr> &joints, const std::vector<ModelLinkPtr> &bodies, std::vector<Gravity::GravityDataPtr> &dataVec)
{
    GravityDataPtr p(new GravityData);
    p->init(node, joints, bodies, dataVec);
    return p;
}

void Gravity::GravityData::init(ModelNodePtr node, const std::vector<ModelJointPtr> &joints, const std::vector<ModelLinkPtr> &bodies, std::vector<GravityDataPtr> &dataVec)
{
    this->node = node;
    nodeJoint = std::dynamic_pointer_cast<ModelJoint>(node);
    nodeLink = std::dynamic_pointer_cast<ModelLink>(node);
    dataVec.resize(joints.size());

    RobotNodePtr thisNode = node;//std::dynamic_pointer_cast<RobotNode>(node);
    for (size_t i = 0; i < joints.size(); ++i) {
        if(thisNode == joints.at(i)){
            computeTorque = true;
            dataVec[i] = shared_from_this();
        }
    }

    for (size_t i = 0; i < bodies.size(); ++i) {
        if(thisNode == bodies.at(i)){
            computeCoM = true;
            //            VR_INFO << "Computing com for " << thisNode->getName() << std::endl;
        }
    }

    if(computeCoM)
    {
        if (nodeLink)
            massSum = nodeLink->getMass();
    } else
        computeCoM = false;
    //    for(auto& body : bodies)
    for(auto child : node->getChildNodes())
    {
        GravityDataPtr data;
        data = GravityData::create(child, joints, bodies, dataVec);
        children[child->getName()] = data;
        //auto childNode = std::dynamic_pointer_cast<RobotNode>(child);
        //                    VR_INFO << "adding child mass sum: " << child->getName() << " : " << data->massSum << std::endl;
        massSum += data->massSum;
    }
    //    if(computeCoM)
    //        VR_INFO << node->getName() << " - body mass: " << node->getMass() << " sum: " << massSum<< std::endl;


}

void Gravity::GravityData::computeCoMAndTorque(Eigen::Vector3f &comPositionGlobal)
{
    comPositionGlobal = Eigen::Vector3f::Zero();
    Eigen::Vector3f comPositionGlobalChild;
    //    std::string name = node->getName();
    float validationSum = computeCoM ?nodeLink->getMass() : 0;
    if(computeCoM && massSum > 0 && nodeLink->getMass() > 0)
        comPositionGlobal += nodeLink->getCoMGlobal() * nodeLink->getMass()/massSum;
    for(auto& child : children)
    {
        child.second->computeCoMAndTorque(comPositionGlobalChild);
        //        if(child.second->computeCoM)
        if(massSum > 0)
        {
            //            VR_INFO << "actually Computing com for " << child.second->node->getName() << std::endl;
            if(child.second->massSum > 0)
                comPositionGlobal += comPositionGlobalChild * child.second->massSum/this->massSum;
            validationSum += child.second->massSum;
        }
    }
    //    BOOST_ASSERT_MSG(fabs(validationSum-massSum) < 0.001, std::string(name + ":" + std::to_string(validationSum) + " vs. " + std::to_string(massSum)).c_str());
    //    if(computeCoM && massSum > 0)
    //        VR_INFO << "CoM of " << name << ": " << node->getCoMGlobal() << " accumulated CoM: " << comPositionGlobal << "\nmass: " << node->getMass() << " massSum: " << massSum << std::endl;
    if(computeTorque && nodeJoint)
    {
        VirtualRobot::ModelJointRevolutePtr rnRevolute = std::dynamic_pointer_cast<VirtualRobot::ModelJointRevolute>(nodeJoint);
        Eigen::Vector3f axisGlobal = rnRevolute->getJointRotationAxis();
        VirtualRobot::MathTools::BaseLine<Eigen::Vector3f> l(node->getGlobalPose().block<3,1>(0,3), axisGlobal);
        Eigen::Vector3f pointOnAxis = VirtualRobot::MathTools::nearestPointOnLine<Eigen::Vector3f>(l, comPositionGlobal);
        Eigen::Vector3f r = comPositionGlobal - pointOnAxis; // vector from axis to com (change sign?)
        r *= 0.001f; // mm -> m
        Eigen::Vector3f F(0, 0, -9.81f * massSum);
        torque = (r.cross(F)).dot(axisGlobal);
        // gravity compensation: invert the gravity torque
        torque *= -1.0f;
    }
}
