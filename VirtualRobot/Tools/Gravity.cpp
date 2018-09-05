#include "Gravity.h"


#include "../Nodes/RobotNodeRevolute.h"
#include "../RobotNodeSet.h"
#include "../Robot.h"

using namespace VirtualRobot;

Gravity::Gravity(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rnsJoints, VirtualRobot::RobotNodeSetPtr rnsBodies) :
    robot(robot),
    rns(rnsJoints),
    rnsBodies(rnsBodies)
{
    THROW_VR_EXCEPTION_IF(!robot, "!robot");
    THROW_VR_EXCEPTION_IF(!rns, "!rnsJoints");
    THROW_VR_EXCEPTION_IF(!rnsBodies, "!rnsBodies");
    THROW_VR_EXCEPTION_IF(rns->getSize()==0, "joints node set is empty");


    nodes = rns->getAllRobotNodes();
    nodesBodies = rnsBodies->getAllRobotNodes();

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
            THROW_VR_EXCEPTION("No mass for body '" << body->getName() << "' mass: " << body->getMass());
        }
    }

    gravityDataHelperRoot = GravityData::create(robot->getRootNode(), nodes, nodesBodies, gravityDataHelperVec);   
}

Gravity::~Gravity()
{
}



void Gravity::computeGravityTorque(std::vector<float> &storeValues)
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

Gravity::GravityDataPtr Gravity::GravityData::create(SceneObjectPtr node, const std::vector<RobotNodePtr> &joints, const std::vector<RobotNodePtr> &bodies, std::vector<Gravity::GravityDataPtr> &dataVec)
{
    GravityDataPtr p(new GravityData);
    p->init(node, joints, bodies, dataVec);
    return p;
}

void Gravity::GravityData::init(SceneObjectPtr node, const std::vector<RobotNodePtr> &joints, const std::vector<RobotNodePtr> &bodies, std::vector<GravityDataPtr> &dataVec)
{
    this->node = node;
    dataVec.resize(joints.size());

    RobotNodePtr thisNode = boost::dynamic_pointer_cast<RobotNode>(node);
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
        massSum = node->getMass();
    //    for(auto& body : bodies)
    for(auto child : node->getChildren())
    {
        GravityDataPtr data;
        data = GravityData::create(child, joints, bodies, dataVec);
        children[child->getName()] = data;
        auto childNode = boost::dynamic_pointer_cast<RobotNode>(child);
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
    float validationSum = computeCoM ?node->getMass() : 0;
    if(computeCoM && massSum > 0 && node->getMass() > 0)
        comPositionGlobal += node->getCoMGlobal() * node->getMass()/massSum;
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
    if(computeTorque)
    {
        VirtualRobot::RobotNodeRevolutePtr rnRevolute = boost::dynamic_pointer_cast<VirtualRobot::RobotNodeRevolute>(node);
        Eigen::Vector3f axisGlobal = rnRevolute->getJointRotationAxis();
        VirtualRobot::MathTools::BaseLine<Eigen::Vector3f> l(node->getGlobalPose().block<3,1>(0,3), axisGlobal);
        Eigen::Vector3f pointOnAxis = VirtualRobot::MathTools::nearestPointOnLine<Eigen::Vector3f>(l, comPositionGlobal);
        Eigen::Vector3f r = comPositionGlobal - pointOnAxis; // vector from axis to com (change sign?)
        r *= 0.001f; // mm -> m
        Eigen::Vector3f F(0, 0, -9.81 * massSum);
        torque = (r.cross(F)).dot(axisGlobal);
        // gravity compensation: invert the gravity torque
        torque *= -1.0f;
    }
}
