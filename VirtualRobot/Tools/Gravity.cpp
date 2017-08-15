#include "Gravity.h"


#include "../Nodes/RobotNodeRevolute.h"
#include "../RobotNodeSet.h"

using namespace VirtualRobot;

Gravity::Gravity(VirtualRobot::RobotPtr robot, VirtualRobot::RobotNodeSetPtr rnsJoints, VirtualRobot::RobotNodeSetPtr rnsBodies) :
    robot(robot),
    rns(rnsJoints),
    rnsBodies(rnsBodies)
{
}

Gravity::~Gravity()
{
}

std::map<std::string, float> Gravity::computeGravityTorque()
{
    std::map<std::string, float> torques;
    if (!rns || !robot)
    {
        VR_ERROR << "Gravity compensation not properly set up" << endl;
        return torques;
    }

    std::vector<VirtualRobot::RobotNodePtr> nodes = rns->getAllRobotNodes();
    std::vector<VirtualRobot::RobotNodePtr> nodesBodies = rnsBodies->getAllRobotNodes();

    for (auto& node : nodes)
    {
        if (!node->isRotationalJoint())
        {
            VR_WARNING << "Not a rotational joint:" << node->getName() << endl;
            continue;
        }

        VirtualRobot::RobotNodeRevolutePtr rnRevolute = boost::dynamic_pointer_cast<VirtualRobot::RobotNodeRevolute>(node);
        Eigen::Vector3f axisGlobal = rnRevolute->getJointRotationAxis();
        Eigen::Vector3f jointPosGlobal = rnRevolute->getGlobalPose().block(0, 3, 3, 1);
        axisGlobal.normalize();

        float gravityJoint = 0;

        for (auto& body : nodesBodies)
        {
            if (body->getMass() <= 0)
            {
                VR_WARNING << "No mass for body" << body->getName() << endl;
                continue;
            }

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
    }
    return torques;
}
