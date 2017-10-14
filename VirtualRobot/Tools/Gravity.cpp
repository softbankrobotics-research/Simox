#include "Gravity.h"


#include "../Nodes/RobotNodeRevolute.h"
#include "../RobotNodeSet.h"

using namespace VirtualRobot;

Gravity::Gravity(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rnsJoints, VirtualRobot::RobotNodeSetPtr rnsBodies) :
    robot(robot),
    rns(rnsJoints),
    rnsBodies(rnsBodies)
{
    THROW_VR_EXCEPTION_IF(!robot || !rns || !rnsBodies || rns->getSize()==0, "NULL data");

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
            THROW_VR_EXCEPTION("No mass for body" << body->getName());
        }
    }

    children.resize(nodes.size(), nodesBodies.size());
    // build children map
    unsigned int a = 0;
    unsigned int b = 0;
    for (auto& node : nodes)
    {
        b=0;
        for (auto& body : nodesBodies)
        {
            if (node->hasChild(body, true) || node == body)
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
        VirtualRobot::RobotNodeRevolutePtr rnRevolute = boost::dynamic_pointer_cast<VirtualRobot::RobotNodeRevolute>(node);
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
                Eigen::Vector3f F(0, 0, -9.81);
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
