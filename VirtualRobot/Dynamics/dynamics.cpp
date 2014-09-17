#include <rbdl/rbdl.h>
#include "dynamics.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeFactory.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Units.h>

#include <Eigen/Dense>

#include <math.h>

#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <iostream>

using std::cout;
using std::cin;
using namespace VirtualRobot;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;






VirtualRobot::Dynamics::Dynamics(RobotNodeSetPtr rns) : rns(rns) {
   if(!rns)
       THROW_VR_EXCEPTION("Null data");

   gravity = Vector3d(0,0,-9.81);
   model = boost::shared_ptr<RigidBodyDynamics::Model>(new Model());

   model->gravity = gravity;

   if (!rns->isKinematicChain())
       THROW_VR_EXCEPTION("RobotNodeSet is no kinematic chain!")

   RobotNodePtr root = rns->getKinematicRoot();

   int rootID = Dynamics::toRBDL(model,root);
}

Eigen::VectorXd Dynamics::getInverseDynamics(Eigen::VectorXd q, Eigen::VectorXd qdot, Eigen::VectorXd qddot)
{
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(Dynamics::model->dof_count);
    InverseDynamics(*model.get(),q,qdot,qddot,tau);
    return tau;
}

Eigen::MatrixXd Dynamics::getInertiaMatrix(Eigen::VectorXd q)
{
    Eigen::MatrixXd inertia = Eigen::MatrixXd::Zero(model->dof_count,model->dof_count);
    CompositeRigidBodyAlgorithm(*model.get(),q,inertia);
    return inertia;
}

void Dynamics::setGravity(Eigen::Vector3d gravity)
{
    model->gravity = gravity;
}

int Dynamics::getnDoF()
{
    return model->dof_count;
}


int Dynamics::toRBDL(boost::shared_ptr<RigidBodyDynamics::Model> model, RobotNodePtr node, RobotNodePtr parentNode, int parentID)
{
    int nodeID = parentID;
    // need to define body, joint and spatial transform
    // body first
    float mass = node->getMass();
    Vector3d com = node->getCoMLocal().cast<double>()/1000; // divide by 1000 because Simox defines lengths in mm while the RBDL defines lengths in m
    Matrix3d inertia = node->getInertiaMatrix().cast<double>();
    Body body = Body(mass, com, inertia);


    // spatial transform next
    Eigen::Matrix4d trafo = Eigen::Matrix4d::Identity();

    if(parentNode)
        trafo = node->getTransformationFrom(parentNode).cast<double>();

    Matrix3d spatial_rotation = trafo.block(0,0,3,3);
    Vector3d spatial_translation = trafo.col(3).head(3)/1000;

    SpatialTransform spatial_transform = SpatialTransform(spatial_rotation,spatial_translation);

    // last, joint
    Joint joint = Joint(JointTypeFixed);
    if (node->isRotationalJoint())
    {
        JointType joint_type = JointTypeRevolute;
        boost::shared_ptr<RobotNodeRevolute> rev = boost::dynamic_pointer_cast<RobotNodeRevolute>(node);
        Vector3d joint_axis = rev->getJointRotationAxisInJointCoordSystem().cast<double>();

        joint = Joint(joint_type, joint_axis);
    }
    else if(node->isTranslationalJoint())
    {
        JointType joint_type = JointTypePrismatic;
        boost::shared_ptr<RobotNodePrismatic> prism = boost::dynamic_pointer_cast<RobotNodePrismatic>(node);
        Vector3d joint_axis = prism->getJointTranslationDirectionJointCoordSystem().cast<double>();

        joint = Joint(joint_type,joint_axis);
    }

    if (joint.mJointType != JointTypeFixed)
    {
        nodeID = model->AddBody(parentID,spatial_transform,joint,body);
    }

    BOOST_FOREACH(SceneObjectPtr child, node->getChildren())
    {
        boost::shared_ptr<RobotNode> childRobotNode = boost::dynamic_pointer_cast<RobotNode>(child);
        if (childRobotNode != 0 && Dynamics::rns->hasRobotNode(childRobotNode)) // if cast returns 0 pointer, child is a sensor and can be omitted. also, child must be contained in the robotnodeset
        {
            if (joint.mJointType == JointTypeFixed) // if current node is fixed, make its parent the parent of the next recursion step and thereby skip it
                node = parentNode;
            toRBDL(model, childRobotNode,node, nodeID);

        }
    }

    return nodeID;
}
