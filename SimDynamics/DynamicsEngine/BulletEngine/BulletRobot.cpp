#include "BulletRobot.h"
#include "BulletEngine.h"
#include "BulletEngineFactory.h"
#include "../../DynamicsWorld.h"
#include "../DynamicsObject.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Model/Obstacle.h>
#include <VirtualRobot/Model/Nodes/ModelJointFixed.h>
#include <VirtualRobot/Model/Nodes/ModelJointPrismatic.h>
#include <VirtualRobot/Model/Nodes/ModelJointRevolute.h>
#include <VirtualRobot/Model/LinkSet.h>
#include <VirtualRobot/Model/JointSet.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Model/Nodes/Attachments/ForceTorqueSensor.h>
//#include <VirtualRobot/Nodes/ContactSensor.h>

// either hinge or generic6DOF constraints can be used
//#define USE_BULLET_GENERIC_6DOF_CONSTRAINT

//#define DEBUG_FIXED_OBJECTS
//#define DEBUG_SHOW_LINKS
using namespace VirtualRobot;
using namespace std;

//#define PRINT_TEST_DEBUG_MESSAGES

namespace SimDynamics
{

    BulletRobot::BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors)
        : DynamicsRobot(rob)
        // should be enough for up to 10ms/step
        , bulletMaxMotorImulse(30 * BulletObject::ScaleFactor)
    {
        ignoreTranslationalJoints = false;

        buildBulletModels(enableJointMotors);

        // activate force torque sensors
        std::vector<SensorPtr>::iterator it = sensors.begin();

        for (; it != sensors.end(); it++)
        {
            ForceTorqueSensorPtr ftSensor = std::dynamic_pointer_cast<ForceTorqueSensor>(*it);

            if (ftSensor)
            {
                VirtualRobot::ModelNodePtr node = ftSensor->getParent();
                THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to ModelNode")
                VirtualRobot::ModelJointPtr nodeJ = std::dynamic_pointer_cast<ModelJoint>(node);
                THROW_VR_EXCEPTION_IF(!nodeJ, "parent of sensor could not be casted to ModelJoint")

                if (!hasLink(nodeJ))
                {
                    VR_WARNING << "Ignoring FT sensor " << ftSensor->getName() << ". Must be linked to a joint" << endl;
                } else
                {
                    const LinkInfo& link = getLink(nodeJ);
                    enableForceTorqueFeedback(link);
                    std::cout << "Found force torque sensor: " << node->getName() << std::endl;
                }
            }
        }


        this->enableSelfCollisions(false);
    }

    BulletRobot::~BulletRobot()
    {
    }



    void BulletRobot::buildBulletModels(bool /*enableJointMotors*/)
    {
        MutexLockPtr lock = getScopedLock();

        if (!robot)
        {
            return;
        }

        jointNodes = robot->getJoints();
        linkNodes = robot->getLinks();


        for (size_t i = 0; i < linkNodes.size(); i++)
        {

            ModelLinkPtr rn = linkNodes[i];
            CollisionModelPtr colModel = rn->getCollisionModel();

            if (colModel)
            {
                addIgnoredCollisionModels(rn);
                // search joint and connected model
                ModelLinkPtr bodyA;
                ModelLinkPtr bodyB = rn;
                ModelJointPtr joint;
                //RobotNodePtr joint2;

                /*if ( (rn->isTranslationalJoint() && !ignoreTranslationalJoints) || rn->isRotationalJoint())
                {
                    joint = rn;
                }*/

                RobotNodePtr parent = rn->getParentNode();

                while (parent && !bodyA)
                {
                    ModelJointPtr j = std::dynamic_pointer_cast<ModelJoint>(parent);
                    if (j && ( (j->getType() == ModelNode::JointPrismatic && !ignoreTranslationalJoints) || j->getType() == ModelNode::JointRevolute))
                    {
                        if (!joint)
                        {
                            joint = j;
                        } else
                        {
                            VR_WARNING << "No body between " << parent->getName() << " and " << joint->getName() << ", skipping " << parent->getName() << endl;
                        }
                    }

                    ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(parent);
                    if (l && l->getCollisionModel())
                    {
                        bodyA = l;
                        break;
                    }

                    parent = parent->getParentNode();
                }

                if (!bodyA)
                {
                    bodyA = robot->getFirstLink();
                }

                // check for fixed joint
                /*if (!joint)
                {
                    joint = bodyB;
                }*/

                createLink(bodyA, joint, /*joint2,*/ bodyB);
            }

        }

    }

    void BulletRobot::addIgnoredCollisionModels(ModelLinkPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!rn->getCollisionModel() || !rn->getCollisionModel()->getTriMeshModel() || !rn->getCollisionModel()->getTriMeshModel()->faces.size()==0)
        {
            return;    // nothing to do: no col model -> no bullet model -> no collisions
        }

        createDynamicsNode(rn);
        std::vector<std::string> ic = rn->getIgnoredCollisionModels();
        RobotPtr robot = rn->getModel();
        BulletObjectPtr drn1 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn]);
        VR_ASSERT(drn1);

        for (size_t i = 0; i < ic.size(); i++)
        {
            ModelLinkPtr rn2 = robot->getLink(ic[i]);

            if (!rn2)
            {
                VR_ERROR << "Error while processing robot node <" << rn->getName() << ">: Ignored collision model <" << ic[i] << "> is not part of robot..." << endl;
            }
            else
            {
                createDynamicsNode(rn2);
                BulletObjectPtr drn2 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[rn2]);
                VR_ASSERT(drn2);
                DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(), drn2.get());
            }
        }
    }

    bool BulletRobot::removeLink(const BulletRobot::LinkInfo& l)
    {
        bool result = false;
        std::vector<LinkInfo>::iterator it = links.begin();

        while (it != links.end())
        {
            if (it->dynNode1 == l.dynNode1 && it->dynNode2 == l.dynNode2)
            {
                it = links.erase(it);
                result = true;
            }
            else
            {
                it++;
            }
        }

        return result;
    }

    std::shared_ptr<btTypedConstraint> BulletRobot::createHingeJoint(std::shared_ptr<btRigidBody> btBody1, std::shared_ptr<btRigidBody> btBody2, Eigen::Matrix4f& coordSystemNode1, Eigen::Matrix4f& coordSystemNode2,  Eigen::Matrix4f& anchor_inNode1, Eigen::Matrix4f& anchor_inNode2, Eigen::Vector3f& /*axisGlobal*/, Eigen::Vector3f& axisLocal, Eigen::Matrix4f& coordSystemJoint, double limMinBT, double limMaxBT)
    {
        // HINGE joint
        /*Eigen::Matrix4f tmpGp1 = coordSystemNode1;
        tmpGp1.block(0,3,3,1).setZero();
        Eigen::Matrix4f tmpGp2 = coordSystemNode2;
        tmpGp2.block(0,3,3,1).setZero();*/
        //Eigen::Vector3f axis_inLocal1 = (tmpGp1.inverse() * axisGlobal).block(0,0,3,1);
        //Eigen::Vector3f axis_inLocal2 = (tmpGp2.inverse() * axisGlobal).block(0,0,3,1);
        //Eigen::Vector3f axis_inLocal2 = rnRev2->getJointRotationAxisInJointCoordSystem();

        btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0, 3, 3, 1));
        btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0, 3, 3, 1));



#ifdef USE_BULLET_GENERIC_6DOF_CONSTRAINT
        THROW_VR_EXCEPTION("USE_BULLET_GENERIC_6DOF_CONSTRAINT nyi in this method...");
#endif

        // we need to align coord system joint, so that z-axis is rotation axis

        MathTools::Quaternion q1 = MathTools::getRotation(Eigen::Vector3f::UnitZ(), axisLocal);
        Eigen::Matrix4f rotationzAlignment = MathTools::quat2eigen4f(q1);
        Eigen::Matrix4f coordSystemJoint_zAligned =  coordSystemJoint * rotationzAlignment ;

        // get transformation coord1 -> joint coord
        Eigen::Matrix4f trafoNode1 = coordSystemNode1.inverse() * coordSystemJoint_zAligned;
        // get transformation coord2 -> joint coord
        Eigen::Matrix4f trafoNode2 = coordSystemNode2.inverse() * coordSystemJoint_zAligned;

        // now we need to pivot points in local coord systems
        btTransform tr1 = BulletEngine::getPoseBullet(trafoNode1);
        btTransform tr2 = BulletEngine::getPoseBullet(trafoNode2);
        tr1.getOrigin() = pivot1;
        tr2.getOrigin() = pivot2;

        std::shared_ptr<btHingeConstraint> hinge(new btHingeConstraint(*btBody1, *btBody2, tr1, tr2, true));
//        hinge->setDbgDrawSize(0.15);

        // todo: check effects of parameters...
//        hinge->setParam(BT_CONSTRAINT_STOP_ERP, 2.0f);
//        hinge->setParam(BT_CONSTRAINT_ERP, 2.f);


//        hinge->setParam(BT_CONSTRAINT_CFM,0);
//        hinge->setParam(BT_CONSTRAINT_STOP_CFM,0.f);
        //hinge->setLimit(limMin,limMax);//,btScalar(1.0f));
//        hinge->setParam(BT_CONSTRAINT_CFM,0.0f);
        hinge->setLimit(btScalar(limMinBT), btScalar(limMaxBT));
        return hinge;
    }

    std::shared_ptr<btTypedConstraint> BulletRobot::createTranslationalJoint(std::shared_ptr<btRigidBody> btBody1, std::shared_ptr<btRigidBody> btBody2, Eigen::Matrix4f &coordSystemNode1, Eigen::Matrix4f &coordSystemNode2, Eigen::Matrix4f& anchor_inNode1, Eigen::Matrix4f& anchor_inNode2, Eigen::Vector3f &directionLocal, Eigen::Matrix4f& coordSystemJoint, double limMinBT, double limMaxBT)
    {
        // we need to align coord system joint, so that z-axis is rotation axis

        btVector3 pivot1 = BulletEngine::getVecBullet(anchor_inNode1.block(0, 3, 3, 1));
        btVector3 pivot2 = BulletEngine::getVecBullet(anchor_inNode2.block(0, 3, 3, 1));

        MathTools::Quaternion q1 = MathTools::getRotation(Eigen::Vector3f::UnitX(), directionLocal); // dont ask me why this has to be the X axis...
        Eigen::Matrix4f rotationzAlignment = MathTools::quat2eigen4f(q1);
        Eigen::Matrix4f coordSystemJoint_zAligned =  coordSystemJoint * rotationzAlignment;

        // now we need to pivot points in local coord systems
        btTransform tr1 = BulletEngine::getPoseBullet(coordSystemNode1.inverse() * coordSystemJoint_zAligned);
        btTransform tr2 = BulletEngine::getPoseBullet(coordSystemNode2.inverse() * coordSystemJoint_zAligned);
        tr1.getOrigin() = pivot1;
        tr2.getOrigin() = pivot2;

        std::shared_ptr<btSliderConstraint> joint(new btSliderConstraint(*btBody1, *btBody2, tr1, tr2, true));

        // disable agular part
        joint->setLowerAngLimit(btScalar(0));
        joint->setUpperAngLimit(btScalar(0));

        joint->setLowerLinLimit(btScalar(limMinBT));
        joint->setUpperLinLimit(btScalar(limMaxBT));

        return joint;
    }

    std::shared_ptr<btTypedConstraint> BulletRobot::createFixedJoint(std::shared_ptr<btRigidBody> btBody1, std::shared_ptr<btRigidBody> btBody2, Eigen::Matrix4f& anchor_inNode1, Eigen::Matrix4f& anchor_inNode2)
    {
        btTransform localA, localB;
        localA = BulletEngine::getPoseBullet(anchor_inNode1);
        localB = BulletEngine::getPoseBullet(anchor_inNode2);
        std::shared_ptr<btGeneric6DofConstraint> generic6Dof(new btGeneric6DofConstraint(*btBody1, *btBody2, localA, localB, true));
        generic6Dof->setOverrideNumSolverIterations(100);

        for (int i = 0; i < 6; i++)
        {
            generic6Dof->setLimit(i, 0, 0);
        }

        /*generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,0);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,1);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,2);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,3);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,4);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_CFM,0.01f,5);*/
        /*generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,0);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,1);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,2);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,3);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,4);
        generic6Dof->setParam(BT_CONSTRAINT_STOP_ERP,0.9f,5);*/
        return generic6Dof;
    }


    void BulletRobot::createLink(VirtualRobot::ModelLinkPtr bodyA, VirtualRobot::ModelJointPtr joint, VirtualRobot::ModelLinkPtr bodyB, bool enableJointMotors)
    {
        MutexLockPtr lock = getScopedLock();

        VR_ASSERT(bodyA && bodyB);

        // ensure dynamics nodes are created
        createDynamicsNode(bodyA);
        createDynamicsNode(bodyB);
        if (hasLink(bodyA, bodyB))
        {
            THROW_VR_EXCEPTION("Joints are already connected:" << bodyA->getName() << "," << bodyB->getName());
        }

        BulletObjectPtr drn1 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyA]);
        BulletObjectPtr drn2 = std::dynamic_pointer_cast<BulletObject>(dynamicRobotNodes[bodyB]);
        VR_ASSERT(drn1);
        VR_ASSERT(drn2);
        std::shared_ptr<btRigidBody> btBody1 = drn1->getRigidBody();
        std::shared_ptr<btRigidBody> btBody2 = drn2->getRigidBody();
        VR_ASSERT(btBody1);
        VR_ASSERT(btBody2);
        DynamicsWorld::GetWorld()->getEngine()->disableCollision(drn1.get(),drn2.get());

        if (joint && joint->getJointValue()!=0.0f)
        {
            VR_WARNING << joint->getName() << ": joint values != 0 may produce a wrong setup, setting joint value to zero" << endl;
            joint->setJointValue(0);
        }

        Eigen::Matrix4f coordSystemNode1 = bodyA->getGlobalPose(); // todo: what if joint is not at 0 ?!
        Eigen::Matrix4f coordSystemNode2 = bodyB->getGlobalPose();

        Eigen::Matrix4f coordSystemJoint = joint?joint->getGlobalPose():bodyB->getGlobalPose();

        Eigen::Matrix4f anchorPointGlobal = coordSystemJoint;// joint->getGlobalPose();//node1->getGlobalPose() * node2->getPreJointTransformation(); //

        Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal;
        Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal;


        // The bullet model was adjusted, so that origin is at local com
        // since we computed the anchor in from simox models, we must re-adjust the anchor, in order to consider the com displacement
        Eigen::Matrix4f com1;
        com1.setIdentity();
        com1.block(0, 3, 3, 1) = -drn1->getCom();
        anchor_inNode1 = com1 * anchor_inNode1;

        Eigen::Matrix4f com2;
        com2.setIdentity();
        com2.block(0, 3, 3, 1) = -drn2->getCom();
        anchor_inNode2 = com2 * anchor_inNode2;

        std::shared_ptr<btTypedConstraint> jointbt;

        double vr2bulletOffset = 0.0f;


        if (joint && joint->isTranslationalJoint() && !ignoreTranslationalJoints)
        {
            std::shared_ptr<ModelJointPrismatic> rnPrisJoint = std::dynamic_pointer_cast<ModelJointPrismatic>(joint);


            double limMin, limMax;
            btScalar diff = joint->getJointValueOffset();
            limMin = (joint->getJointLimitLow() + diff) / 1000 * BulletObject::ScaleFactor; //mm -> m
            limMax = (joint->getJointLimitHigh() + diff) / 1000 * BulletObject::ScaleFactor; //mm -> m

            Eigen::Vector3f directionLocal = rnPrisJoint->getJointTranslationDirectionJointCoordSystem();

            jointbt = createTranslationalJoint(btBody1, btBody2, coordSystemNode1, coordSystemNode2, anchor_inNode1, anchor_inNode2, directionLocal, coordSystemJoint, limMin, limMax);

            vr2bulletOffset = diff;
        }
        else if (joint && joint->isRotationalJoint())
        {
            // create joint
            std::shared_ptr<ModelJointRevolute> rnRevJoint = std::dynamic_pointer_cast<ModelJointRevolute>(joint);

            // transform axis direction (not position!)
            Eigen::Vector4f axisLocalJoint = Eigen::Vector4f::Zero();
            axisLocalJoint.block(0, 0, 3, 1) =  rnRevJoint->getJointRotationAxisInJointCoordSystem();
            Eigen::Matrix4f tmpGpJoint = coordSystemJoint;
            tmpGpJoint.block(0, 3, 3, 1).setZero(); // coordSystemJoint
            //Eigen::Vector4f axisGlobal = tmpGpJoint * axisLocalJoint;

            double limMin, limMax;
            limMin = joint->getJointLimitLow();
            limMax = joint->getJointLimitHigh();

            Eigen::Vector3f axisGlobal = rnRevJoint->getJointRotationAxis();
            Eigen::Vector3f axisLocal = rnRevJoint->getJointRotationAxisInJointCoordSystem();
            btScalar limMinBT, limMaxBT;
            btScalar diff = joint->getJointValueOffset();//startAngleBT + startAngle);
            limMinBT = btScalar(limMin) + diff;//diff - limMax;//
            limMaxBT = btScalar(limMax) + diff;//diff - limMin;//
            jointbt = createHingeJoint(btBody1, btBody2, coordSystemNode1, coordSystemNode2, anchor_inNode1, anchor_inNode2, axisGlobal, axisLocal, coordSystemJoint, limMinBT, limMaxBT);

            vr2bulletOffset = diff;
        }
        else
        {
            VR_WARNING << "Creating fixed joint between " << bodyA->getName() << " and " << bodyB->getName() << ". This might result in some artefacts (e.g. no strict ridgid connection)" << endl;
            // create fixed joint
            jointbt = createFixedJoint(btBody1, btBody2, anchor_inNode1, anchor_inNode2);
        }

        LinkInfo i;
        i.nodeA = bodyA;
        i.nodeB = bodyB;
        i.dynNode1 = drn1;
        i.dynNode2 = drn2;
        i.nodeJoint = joint;
        i.joint = jointbt;
        i.jointValueOffset = vr2bulletOffset;

        // activate joint feedback for translational joints (avoids jumping positions?)
        if (joint && joint->isTranslationalJoint() && !ignoreTranslationalJoints)
            enableForceTorqueFeedback(i);

        // disable col model
        i.disabledCollisionPairs.push_back(
            std::pair<DynamicsObjectPtr, DynamicsObjectPtr>(
                std::dynamic_pointer_cast<DynamicsObject>(drn1),
                std::dynamic_pointer_cast<DynamicsObject>(drn2)));

        links.push_back(i);
#ifndef DEBUG_FIXED_OBJECTS

        if (enableJointMotors && joint && ((joint->getType() == ModelNode::JointRevolute) || ((joint->getType() == ModelNode::JointPrismatic) && !ignoreTranslationalJoints)))
        {
            // start standard actuator
            actuateNode(joint, joint->getJointValue());
        }

#endif
    }

    bool BulletRobot::hasLink(VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2)
    {
        MutexLockPtr lock = getScopedLock();

        for (size_t i = 0; i < links.size(); i++)
        {
            if (links[i].nodeA == node1 && links[i].nodeB == node2)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<BulletRobot::LinkInfo> BulletRobot::getLinks()
    {
        return links;
    }

    void BulletRobot::ensureKinematicConstraints()
    {
        // results in strange behavior?!
#if 0
        // update globalpose of robot
        Eigen::Matrix4f gpRoot = robot->getRootNode()->getGlobalPoseVisualization();
        Eigen::Matrix4f rootPreJoint = robot->getRootNode()->getPreJointTransformation();
        robot->setGlobalPose(gpRoot * rootPreJoint.inverse());
        //robot->applyJointValues();
        std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr>::iterator i = dynamicRobotNodes.begin();

        while (i != dynamicRobotNodes.end())
        {
            i->second->setPose(i->first->getGlobalPoseVisualization());
            i++;
        }

#endif
    }

    void BulletRobot::actuateJoints(double dt)
    {
        MutexLockPtr lock = getScopedLock();
        //cout << "=== === BulletRobot: actuateJoints() 1 === " << this << endl;

        auto  it = actuationTargets.begin();

        //int jointCounter = 0;
        //cout << "**** Control Values: ";

        for (; it != actuationTargets.end(); it++)
        {
            //cout << "it:" << it->first << ", name: " << it->first->getName() << endl;
            VelocityMotorController& controller = actuationControllers[it->first];

            if (!it->second.node->isRotationalJoint() && !it->second.node->isTranslationalJoint())
                continue;

            LinkInfo link = getLink(it->second.node);

            const ActuationMode& actuation = it->second.actuation;

            // CHECK FOR DISABLED MOTORS
            if (actuation.mode == 0)
            {
                if (it->second.node->isRotationalJoint())
                {
                    std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
                    hinge->enableMotor(false);
                    continue;
                }
                else if (it->second.node->isTranslationalJoint() && !ignoreTranslationalJoints)
                {
                    std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
                    slider->setPoweredLinMotor(false);
                    continue;
                }
             }


            if (actuation.modes.torque)
            {
                // TORQUE MODES

                 if (it->second.node->isRotationalJoint())
                 {
                     std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
                     auto torque = it->second.jointTorqueTarget;
                     btVector3 hingeAxisLocalA =
                             hinge->getFrameOffsetA().getBasis().getColumn(2);
                     btVector3 hingeAxisLocalB =
                             hinge->getFrameOffsetB().getBasis().getColumn(2);
                     btVector3 hingeAxisWorldA =
                             hinge->getRigidBodyA().getWorldTransform().getBasis() *
                             hingeAxisLocalA;
                     btVector3 hingeAxisWorldB =
                             hinge->getRigidBodyB().getWorldTransform().getBasis() *
                             hingeAxisLocalB;


                     int sign = torque > 0?1:-1;
                     torque = std::min<double>(fabs(torque), it->first->getMaxTorque()) * sign;

                     btVector3 hingeTorqueA = - torque * hingeAxisWorldA;
                     btVector3 hingeTorqueB =   torque * hingeAxisWorldB;
                     hinge->enableMotor(false);
                     hinge->getRigidBodyA().applyTorque(hingeTorqueA);
                     hinge->getRigidBodyB().applyTorque(hingeTorqueB);
                 }
                 else
                 {
                     // not yet tested!
                     std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
                     auto torque = it->second.jointTorqueTarget;
                     btVector3 sliderAxisLocalA =
                             slider->getFrameOffsetA().getBasis().getColumn(2);
                     btVector3 sliderAxisLocalB =
                             slider->getFrameOffsetB().getBasis().getColumn(2);
                     btVector3 sliderAxisWorldA =
                             slider->getRigidBodyA().getWorldTransform().getBasis() *
                             sliderAxisLocalA;
                     btVector3 sliderAxisWorldB =
                             slider->getRigidBodyB().getWorldTransform().getBasis() *
                             sliderAxisLocalB;


                     int sign = torque > 0?1:-1;
                     torque = std::min<double>(fabs(torque), it->first->getMaxTorque()) * sign;

                     btVector3 sliderTorqueA = - torque * sliderAxisWorldA;
                     btVector3 sliderTorqueB =   torque * sliderAxisWorldB;
                     slider->setPoweredLinMotor(false);
                     slider->btTypedConstraint::getRigidBodyA().applyCentralForce(sliderTorqueA);
                     slider->btTypedConstraint::getRigidBodyB().applyCentralForce(sliderTorqueB);
                 }
            }
            else if (actuation.modes.position || actuation.modes.velocity)
            {
                // POSITION, VELOCITY OR POSITION&VELOCITY MODE

                btScalar velActual = btScalar(getJointSpeed(it->first));
                btScalar velocityTarget = btScalar(it->second.jointVelocityTarget);

                if (actuation.modes.velocity && !actuation.modes.position)
                {
                    // bullet is buggy here and cannot reach velocity targets for some joints, use a position-velocity mode as workaround
                    it->second.jointValueTarget += velocityTarget * dt;
                }

                btScalar posTarget = btScalar(it->second.jointValueTarget + link.jointValueOffset);
                btScalar posActual = btScalar(getJointAngle(it->first));
                controller.setName(it->first->getName());

                double targetVelocity;
                float deltaPos = it->second.node->getDelta(posTarget);
                if (it->second.node->isTranslationalJoint())
                {
                    posTarget *= 0.001;
                    posActual *= 0.001;
                    velActual *= 0.001;
                    velocityTarget *= 0.001;
                    deltaPos *= 0.001;
                }
                if (!actuation.modes.position)
                {
                    // we always use position or position-velocity mode
                    ActuationMode tempAct = actuation;
                    tempAct.modes.position = 1;
                    targetVelocity = controller.update(/*posTarget - posActual*/ deltaPos, velocityTarget, tempAct, btScalar(dt));
                }
                else
                {
                    targetVelocity = controller.update(/*posTarget - posActual*/ deltaPos, velocityTarget, actuation, btScalar(dt));
                }

                btScalar maxImpulse = bulletMaxMotorImulse;
//                controller.setCurrentVelocity(velActual);
                if (it->second.node->getMaxTorque() > 0)
                {
                    maxImpulse = it->second.node->getMaxTorque() * btScalar(dt) * BulletObject::ScaleFactor  * BulletObject::ScaleFactor * BulletObject::ScaleFactor;
                    //cout << "node:" << it->second.node->getName() << ", max impulse: " << maxImpulse << ", dt:" << dt << ", maxImp:" << it->second.node->getMaxTorque() << endl;
                }
#ifdef PRINT_TEST_DEBUG_MESSAGES

                if (it->first->getName() == "LegL_Ank1_joint")
//                if(posTarget - posActual > 0.05)
                {
                    double p,i,d;
                    controller.getPosPID(p,i,d);
                    std::cout << it->first->getName() << ": " << (actuation.modes.velocity?true:false) << " " << (actuation.modes.position?true:false) << " p: " << p;
                    cout << "################### " << it->first->getName() <<" error: " << (posTarget - posActual)<< ", actvel:"  << velActual << ", target vel:" << targetVelocity << ", maxImpulse: " << maxImpulse << " applied impulse " << hinge->internalGetAppliedImpulse() << endl;
                }

#endif
                if(fabs(targetVelocity) > 0.00001)
                {
                    link.dynNode1->getRigidBody()->activate();
                    link.dynNode2->getRigidBody()->activate();
                }
                if (it->second.node->isRotationalJoint())
                {
                    std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
                    hinge->enableAngularMotor(true, btScalar(targetVelocity), maxImpulse);
                }
                else if (it->second.node->isTranslationalJoint() && !ignoreTranslationalJoints)
                {
                    std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
                    slider->setMaxLinMotorForce(maxImpulse * 1000); // Magic number!!!
                    slider->setTargetLinMotorVelocity(btScalar(targetVelocity));
                    slider->setPoweredLinMotor(true);
                    //cout << "targetVelocity:" << targetVelocity << "\t posTarget: " << posTarget << "\t posActual:" << posActual << "\t velActual: " << velActual << "\t deltaPos: " << deltaPos << endl;
                }
            }
        }

        //cout << endl;

        // now done in updateVisualization
        //setPoseNonActuatedRobotNodes();
    }

    void BulletRobot::updateSensors(double dt)
    {
        MutexLockPtr lock = getScopedLock();
        //todo: contact sensors
        /*
        std::unordered_set<std::string> contactObjectNames;

        // this seems stupid and it is, but that is abstract interfaces for you.
        for (std::vector<SensorPtr>::iterator it = sensors.begin(); it != sensors.end(); it++)
        {
            ContactSensorPtr contactSensor = std::dynamic_pointer_cast<ContactSensor>(*it);

            if (contactSensor)
            {
                contactObjectNames.insert(contactSensor->getRobotNode()->getName());
            }
        }

        DynamicsWorldPtr world = DynamicsWorld::GetWorld();
        std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo> contacts = world->getEngine()->getContacts();
        boost::unordered_map<std::string, VirtualRobot::ContactSensor::ContactFrame> frameMap;

        for (std::vector<SimDynamics::DynamicsEngine::DynamicsContactInfo>::iterator it = contacts.begin();
             it != contacts.end(); it++)
        {
            const SimDynamics::DynamicsEngine::DynamicsContactInfo& contact = *it;

            if (contact.objectAName.empty() || contact.objectBName.empty())
            {
                continue;
            }

            float sign;
            std::string key;
            std::string contactBody;

            if (contactObjectNames.find(contact.objectAName) != contactObjectNames.end())
            {
                sign = 1.0f;
                key = contact.objectAName;
                contactBody = contact.objectBName;
            }
            else if (contactObjectNames.find(contact.objectBName) != contactObjectNames.end())
            {
                sign = -1.0f;
                key = contact.objectBName;
                contactBody = contact.objectAName;
            }
            else
            {
                continue;
            }

            VirtualRobot::ContactSensor::ContactFrame& frame = frameMap[key];
            double zForce = sign * contact.normalGlobalB.z() * contact.appliedImpulse;
            VirtualRobot::ContactSensor::ContactForce cf;
            cf.contactPoint = contact.posGlobalB;
            cf.zForce = zForce;
            cf.bodyName = contactBody;
            frame.forces.push_back(cf);
        }
        */

        // Update forces and torques
        for (std::vector<SensorPtr>::iterator it = sensors.begin(); it != sensors.end(); it++)
        {
            ForceTorqueSensorPtr ftSensor = std::dynamic_pointer_cast<ForceTorqueSensor>(*it);

            if (ftSensor)
            {
                VirtualRobot::ModelNodePtr node = ftSensor->getParent();
                THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")
                VirtualRobot::ModelJointPtr nodeJ = std::dynamic_pointer_cast<ModelJoint>(node);
                THROW_VR_EXCEPTION_IF(!nodeJ, "parent of sensor could not be casted to ModelJoint")

                const LinkInfo& link = getLink(nodeJ);
                Eigen::VectorXf forceTorques = getJointForceTorqueGlobal(link);
                ftSensor->updateSensors(forceTorques);
            }
            // todo: contact sensor
            /*else
            {
                ContactSensorPtr contactSensor = std::dynamic_pointer_cast<ContactSensor>(*it);

                if (contactSensor)
                {
                    VirtualRobot::RobotNodePtr node = contactSensor->getRobotNode();
                    THROW_VR_EXCEPTION_IF(!node, "parent of sensor could not be casted to RobotNode")
                    const VirtualRobot::ContactSensor::ContactFrame& frame = frameMap[node->getName()];
                    contactSensor->updateSensors(frame, dt);
                }
            }*/
        }
    }

    BulletRobot::LinkInfo BulletRobot::getLink(VirtualRobot::ModelJointPtr node)
    {
        MutexLockPtr lock = getScopedLock();

        for (size_t i = 0; i < links.size(); i++)
        {
            if (links[i].nodeJoint == node /*|| links[i].nodeJoint2 == node*/)
            {
                return links[i];
            }
        }

        THROW_VR_EXCEPTION("No link with node " << node->getName());
        return LinkInfo();
    }

    BulletRobot::LinkInfo BulletRobot::getLink(BulletObjectPtr object1, BulletObjectPtr object2)
    {
        MutexLockPtr lock = getScopedLock();

        for (size_t i = 0; i < links.size(); i++)
        {
            if ((links[i].dynNode1 == object1 && links[i].dynNode2 == object2) || (links[i].dynNode1 == object2 && links[i].dynNode2 == object1))
            {
                return links[i];
            }
        }

        VR_WARNING << "No link with nodes: " << object1->getName() << " and " << object2->getName() << endl;
        return LinkInfo();
    }

    std::vector<BulletRobot::LinkInfo> BulletRobot::getLinks(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<BulletRobot::LinkInfo> result;

        for (size_t i = 0; i < links.size(); i++)
        {
            if (links[i].nodeJoint == node /*|| links[i].nodeJoint2 == node*/ || links[i].nodeA == node || links[i].nodeB == node)
            {
                result.push_back(links[i]);
            }
        }

        return result;
    }

    std::vector<BulletRobot::LinkInfo> BulletRobot::getLinks(BulletObjectPtr node)
    {
        MutexLockPtr lock = getScopedLock();
        std::vector<BulletRobot::LinkInfo> result;

        for (size_t i = 0; i < links.size(); i++)
        {
            if (links[i].dynNode1 == node || links[i].dynNode2 == node)
            {
                result.push_back(links[i]);
            }
        }

        return result;

    }

    bool BulletRobot::attachObject(const string& nodeName, DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr li = attachObjectLink(nodeName, object);
        return bool(li);
    }

    BulletRobot::LinkInfoPtr BulletRobot::attachObjectLink(const string& nodeName, DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr result;

        if (!robot || !robot->hasLink(nodeName))
        {
            VR_ERROR << "no link with name " << nodeName << endl;
            return result;
        }

        if (!object)
        {
            VR_ERROR << "no object " << endl;
            return result;
        }

        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(object);

        if (!bo)
        {
            VR_ERROR << "no bullet object " << endl;
            return result;
        }

        ModelLinkPtr nodeA = robot->getLink(nodeName);

        DynamicsObjectPtr drn = getDynamicsRobotNode(nodeA);

        if (!drn)
        {
            while (nodeA && !drn)
            {
                ModelNodePtr ts = nodeA->getParentNode();
                nodeA = std::dynamic_pointer_cast<ModelLink>(ts);

                if (nodeA)
                {
                    drn = getDynamicsRobotNode(nodeA);
                }
            }

            if (!drn)
            {
                VR_ERROR << "No dynamics object..." << endl;
                return result;
            }
        }

        BulletObjectPtr bdrn = std::dynamic_pointer_cast<BulletObject>(drn);

        if (!bo)
        {
            VR_ERROR << "no bullet robot object " << endl;
            return result;
        }

        // create bullet joint
        std::shared_ptr<btRigidBody> btBody1 = bdrn->getRigidBody();
        std::shared_ptr<btRigidBody> btBody2 = bo->getRigidBody();

        Eigen::Matrix4f coordSystemNode1 = bdrn->getComGlobal(); // todo: what if joint is not at 0 ?!
        Eigen::Matrix4f coordSystemNode2 = bo->getComGlobal();
        //Eigen::Matrix4f coordSystemJoint = bdrn->getComGlobal();

        Eigen::Matrix4f anchorPointGlobal = bdrn->getComGlobal();//node1->getGlobalPose() * node2->getPreJointTransformation(); //

        Eigen::Matrix4f anchor_inNode1 = coordSystemNode1.inverse() * anchorPointGlobal;
        Eigen::Matrix4f anchor_inNode2 = coordSystemNode2.inverse() * anchorPointGlobal;


        // The bullet model was adjusted, so that origin is at local com
        // since we computed the anchor in from simox models, we must re-adjust the anchor, in order to consider the com displacement
        /*Eigen::Matrix4f com1;
        com1.setIdentity();
        com1.block(0,3,3,1) = -drn1->getCom();
        anchor_inNode1 = com1 * anchor_inNode1;

        Eigen::Matrix4f com2;
        com2.setIdentity();
        com2.block(0,3,3,1) = -drn2->getCom();
        anchor_inNode2 = com2 * anchor_inNode2;*/

        std::shared_ptr<btTypedConstraint> jointbt = createFixedJoint(btBody1, btBody2, anchor_inNode1, anchor_inNode2);

        result.reset(new LinkInfo());
        result->nodeA = nodeA;
        //i.nodeB = ;
        result->dynNode1 = bdrn;
        result->dynNode2 = bo;
        //result->nodeJoint = nodeA;

        result->joint = jointbt;
        result->jointValueOffset = 0;

        // disable col model
        result->disabledCollisionPairs.push_back(
            std::pair<DynamicsObjectPtr, DynamicsObjectPtr>(
                drn,
                object));

        links.push_back(*result);

        VR_INFO << "Attached object " << object->getName() << " to node " << nodeName << endl;
        return result;
    }

    bool BulletRobot::detachObject(DynamicsObjectPtr object)
    {
        BulletRobot::LinkInfoPtr result;

        if (!robot)
        {
            VR_ERROR << "no robot " << endl;
            return false;
        }

        if (!object)
        {
            VR_ERROR << "no object " << endl;
            return false;
        }

        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(object);

        if (!bo)
        {
            VR_ERROR << "no bullet object " << endl;
            return false;
        }

        bool res = true;
        std::vector<LinkInfo> ls = getLinks(bo);

        if (ls.size() == 0)
        {
            VR_ERROR << "No link with object " << object->getName() << endl;
            return true; // not a failure, object is not attached
        }

        for (size_t i = 0; i < ls.size(); i++)
        {
            res = res & removeLink(ls[i]);
        }

        VR_INFO << "Detached object " << object->getName() << " from robot " << robot->getName() << endl;
        return res;
    }

    bool BulletRobot::hasLink(VirtualRobot::RobotNodePtr node)
    {
        MutexLockPtr lock = getScopedLock();

        for (size_t i = 0; i < links.size(); i++)
        {
            if (links[i].nodeJoint == node /*|| links[i].nodeJoint2 == node*/)
            {
                return true;
            }
        }

        return false;
    }

    void BulletRobot::actuateNode(VirtualRobot::ModelJointPtr node, double jointValue)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);

        if (node->getType() == ModelNode::JointRevolute || (node->isTranslationalJoint() && !ignoreTranslationalJoints))
        {
            if (!hasLink(node))
            {
                VR_ERROR << "No link for node " << node->getName() << endl;
                return;
            }
            DynamicsRobot::actuateNode(node, jointValue);
        }
        else
        {
            if (node->isTranslationalJoint() && ignoreTranslationalJoints)
            {
                VR_WARNING << "Translational joints ignored. (ignoring node " << node->getName() << ")." << endl;
            }
            else
            {
                VR_ERROR << "Only Revolute and translational joints implemented so far (ignoring node " << node->getName() << ")." << endl;
            }
        }
    }

    void BulletRobot::actuateNodeVel(ModelJointPtr node, double jointVelocity)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(node);
        if (node->getType() == ModelNode::JointRevolute || node->getType() == ModelNode::JointPrismatic)
        {
            if (!hasLink(node))
            {
                VR_ERROR << "No link for node " << node->getName() << endl;
                return;
            }
            DynamicsRobot::actuateNodeVel(node, jointVelocity); // inverted joint direction in bullet
        }
        else
        {
            VR_ERROR << "Only Revolute and Prismatic joints implemented so far (node: " << node->getName() <<")..." << endl;
        }
    }

    double BulletRobot::getJointAngle(ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return 0.0f;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint())
        {
            std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);
            VR_ASSERT(hinge);

            return (hinge->getHingeAngle() - link.jointValueOffset); // inverted joint direction in bullet
        }
        else if (rn->isTranslationalJoint())
        {
            std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);
            VR_ASSERT(slider);

            return (slider->getLinearPos() - link.jointValueOffset) * 1000 / BulletObject::ScaleFactor; // m -> mm
        }
        else
        {
            return 0.0;
        }
    }

    double BulletRobot::getJointTargetSpeed(VirtualRobot::ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return 0.0f;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint())
        {
            std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);

            return hinge->getMotorTargetVelosity();
        }
        else if (rn->isTranslationalJoint())
        {
            std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);

            return slider->getTargetLinMotorVelocity() * 1000; // / BulletObject::ScaleFactor; m -> mm
        }
    }

    double BulletRobot::getJointSpeed(VirtualRobot::ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return 0.0f;
        }

        if (rn->isRotationalJoint())
        {
            LinkInfo link = getLink(rn);
            std::shared_ptr<btHingeConstraint> hinge = std::dynamic_pointer_cast<btHingeConstraint>(link.joint);

            VR_ASSERT(hinge);

            std::shared_ptr<ModelJointRevolute> rnRevJoint = std::dynamic_pointer_cast<ModelJointRevolute>(link.nodeJoint);

            Eigen::Vector3f deltaVel = link.dynNode2->getAngularVelocity() - link.dynNode1->getAngularVelocity();
            double speed = deltaVel.dot(rnRevJoint->getJointRotationAxis());
            return speed;//hinge->getMotorTargetVelosity();
        }
        else if (rn->isTranslationalJoint())
        {
            LinkInfo link = getLink(rn);
            std::shared_ptr<btSliderConstraint> slider = std::dynamic_pointer_cast<btSliderConstraint>(link.joint);

            VR_ASSERT(slider);

            std::shared_ptr<ModelJointPrismatic> rnPrisJoint = std::dynamic_pointer_cast<ModelJointPrismatic>(link.nodeJoint);

            Eigen::Vector3f jointDirection = rnPrisJoint->getGlobalPose().block<3, 3>(0, 0) * rnPrisJoint->getJointTranslationDirectionJointCoordSystem();
            Eigen::Vector3f deltaVel = (link.dynNode2->getLinearVelocity() - link.dynNode1->getLinearVelocity());
            return jointDirection.dot(deltaVel) / jointDirection.norm();
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << endl;
            return 0.0;
        }
    }

    double BulletRobot::getNodeTarget(ModelJointPtr node)
    {
        MutexLockPtr lock = getScopedLock();
        return DynamicsRobot::getNodeTarget(node);
    }

    Eigen::Vector3f BulletRobot::getJointTorques(ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);
        Eigen::Vector3f result;
        result.setZero();

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return result;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            result = getJointForceTorqueGlobal(link).tail(3);
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << endl;
        }

        return result;
    }

    double BulletRobot::getJointTorque(ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return 0.0;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            Eigen::Vector3f torqueVector = getJointForceTorqueGlobal(link).tail(3);

            // project onto joint axis
            double troque = (torqueVector.adjoint() * link.nodeJoint->getGlobalPose().block(0, 2, 3, 1))(0, 0);
            return troque;
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << endl;
        }
        return 0.0;
    }

    Eigen::Vector3f BulletRobot::getJointForces(ModelJointPtr rn)
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(rn);
        Eigen::Vector3f result;
        result.setZero();

        if (!hasLink(rn))
        {
            //VR_ERROR << "No link with node " << rn->getName() << endl;
            return result;
        }

        LinkInfo link = getLink(rn);

        if (rn->isRotationalJoint() || rn->isTranslationalJoint())
        {
            enableForceTorqueFeedback(link, true);
            result = getJointForceTorqueGlobal(link).head(3);
        }
        else
        {
            VR_WARNING << "Only translational and rotational joints implemented." << endl;
        }

        return result;
    }

    Eigen::Matrix4f BulletRobot::getComGlobal(const ModelLinkPtr& rn)
    {
        MutexLockPtr lock = getScopedLock();
        BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(rn));

        if (!bo)
        {
            VR_ERROR << "Could not cast object..." << endl;
            return Eigen::Matrix4f::Identity();
        }

        return bo->getComGlobal();
    }

    Eigen::Vector3f BulletRobot::getComGlobal(const LinkSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f com = Eigen::Vector3f::Zero();
        double totalMass = 0.0;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            ModelLinkPtr node = (*set)[i];
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Matrix4f pose = bo->getComGlobal();
            com += node->getMass() * pose.block(0, 3, 3, 1);
            totalMass += node->getMass();
        }

        com *= float(1.0f / totalMass);
        return com;
    }

    Eigen::Vector3f BulletRobot::getComVelocityGlobal(const LinkSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f com = Eigen::Vector3f::Zero();
        double totalMass = 0.0;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            ModelLinkPtr node = (*set)[i];
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity();

            if (boost::math::isnan(vel(0)) || boost::math::isnan(vel(1)) || boost::math::isnan(vel(2)))
            {
                VR_ERROR << "NAN result: getLinearVelocity:" << bo->getName() << ", i:" << i << endl;
                node->print();
                VR_ERROR << "BULLETOBJECT com:" << bo->getCom() << endl;
                VR_ERROR << "BULLETOBJECT: ang vel:" << bo->getAngularVelocity() << endl;
                VR_ERROR << "BULLETOBJECT:" << bo->getRigidBody()->getWorldTransform().getOrigin() << endl;


            }

            com += node->getMass() * vel;
            totalMass += node->getMass();
        }

        if (fabs(totalMass) < 1e-5)
        {
            VR_ERROR << "Little mass: " << totalMass << ". Could not compute com velocity..." << endl;
        }
        else
        {
            com *= float(1.0f / totalMass);
        }

        return com;
    }

    Eigen::Vector3f BulletRobot::getLinearMomentumGlobal(const LinkSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f linMomentum = Eigen::Vector3f::Zero();

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            ModelLinkPtr node = (*set)[i];
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity() / 1000.0  * BulletObject::ScaleFactor;

            linMomentum += node->getMass() * vel;
        }

        return linMomentum;
    }

    Eigen::Vector3f BulletRobot::getAngularMomentumGlobal(const LinkSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f angMomentum = Eigen::Vector3f::Zero();

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            ModelLinkPtr node = (*set)[i];
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f vel = bo->getLinearVelocity() / 1000.0  * BulletObject::ScaleFactor;
            Eigen::Vector3f ang = bo->getAngularVelocity() / 1000.0  * BulletObject::ScaleFactor;
            Eigen::Vector3f com = bo->getComGlobal().block(0, 3, 3, 1) / 1000.0  * BulletObject::ScaleFactor;
            double mass = node->getMass();

            std::shared_ptr<btRigidBody> body = bo->getRigidBody();
            Eigen::Matrix3f intertiaWorld = BulletEngine::getRotMatrix(body->getInvInertiaTensorWorld()).inverse().block(0, 0, 3, 3);

            angMomentum += com.cross(mass * vel) + intertiaWorld * ang;
        }

        return angMomentum;
    }

    Eigen::Vector3f BulletRobot::getAngularMomentumLocal(const LinkSetPtr& set)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::Vector3f angMomentum = Eigen::Vector3f::Zero();
        Eigen::Vector3f com = getComGlobal(set) / 1000.0  * BulletObject::ScaleFactor;
        Eigen::Vector3f comVel = getComVelocityGlobal(set) / 1000;

        for (unsigned int i = 0; i < set->getSize(); i++)
        {
            ModelLinkPtr node = (*set)[i];
            BulletObjectPtr bo = std::dynamic_pointer_cast<BulletObject>(getDynamicsRobotNode(node));
            Eigen::Vector3f bodyVel = bo->getLinearVelocity() / 1000.0  * BulletObject::ScaleFactor;
            Eigen::Vector3f ang = bo->getAngularVelocity() / 1000.0  * BulletObject::ScaleFactor;
            Eigen::Vector3f bodyCoM = bo->getComGlobal().block(0, 3, 3, 1) / 1000.0  * BulletObject::ScaleFactor;
            double mass = node->getMass();

            std::shared_ptr<btRigidBody> body = bo->getRigidBody();

            btVector3 invIntertiaDiag = body->getInvInertiaDiagLocal();
            Eigen::Matrix3f intertiaLocal = Eigen::Matrix3f::Zero();
            intertiaLocal(0, 0) = 1 / invIntertiaDiag.getX();
            intertiaLocal(1, 1) = 1 / invIntertiaDiag.getY();
            intertiaLocal(2, 2) = 1 / invIntertiaDiag.getZ();

            angMomentum += mass * (bodyCoM - com).cross(bodyVel - comVel) + intertiaLocal * ang;
        }

        return angMomentum;
    }

    void BulletRobot::setPoseNonActuatedRobotNodes()
    {
        MutexLockPtr lock = getScopedLock();
        VR_ASSERT(robot);
        std::vector<RobotNodePtr> rns = robot->getModelNodes();
        std::vector<RobotNodePtr> actuatedNodes;
        std::vector<RobotNodePtr> notActuatedNodes;
        size_t i;

        // only objects with collisionmodel are processed by bullet
        for (i = 0; i < rns.size(); i++)
        {
            ModelLinkPtr l = std::dynamic_pointer_cast<ModelLink>(rns[i]);
            if (l && l->getCollisionModel())
            {
                actuatedNodes.push_back(rns[i]);
            }
            else
            {
                notActuatedNodes.push_back(rns[i]);
            }
        }

        size_t lastSize = notActuatedNodes.size();

        while (notActuatedNodes.size() > 0)
        {
            vector<RobotNodePtr>::iterator it = notActuatedNodes.begin();

            while (it != notActuatedNodes.end())
            {
                ModelNodePtr parent = (*it)->getParentNode();

                if (!parent || find(actuatedNodes.begin(), actuatedNodes.end(), parent) != actuatedNodes.end())
                {
                    // parent is at correct pose, we can update *it
                    if (parent)
                    {
                        (*it)->updatePose(false);
                    }

                    // if root, we also have to delete node from list
                    actuatedNodes.push_back(*it);
                    it = notActuatedNodes.erase(it);
                }
                else
                {
                    it++;
                }
            }


            // just a sanity check
            if (lastSize ==  notActuatedNodes.size())
            {
                VR_ERROR << "Internal error?!" << endl;
                return;
            }
            else
            {
                lastSize =  notActuatedNodes.size();
            }
        }
    }

    void BulletRobot::enableForceTorqueFeedback(const LinkInfo& link , bool enable)
    {
        MutexLockPtr lock = getScopedLock();

        if (!link.joint->needsFeedback() && enable)
        {
            link.joint->enableFeedback(true);
            btJointFeedback* feedback = new btJointFeedback;
            feedback->m_appliedForceBodyA = btVector3(0, 0, 0);
            feedback->m_appliedForceBodyB = btVector3(0, 0, 0);
            feedback->m_appliedTorqueBodyA = btVector3(0, 0, 0);
            feedback->m_appliedTorqueBodyB = btVector3(0, 0, 0);
            link.joint->setJointFeedback(feedback);
        }
        else if (link.joint->needsFeedback() && !enable)
        {
            link.joint->enableFeedback(false);
        }
    }

    Eigen::VectorXf BulletRobot::getForceTorqueFeedbackA(const LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf r(6);
        r.setZero();

        if (!link.joint || !link.joint->needsFeedback())
        {
            return r;
        }

        btJointFeedback* feedback = link.joint->getJointFeedback();

        if (!feedback)
        {
            return r;
        }

        r << feedback->m_appliedForceBodyA[0], feedback->m_appliedForceBodyA[1], feedback->m_appliedForceBodyA[2], feedback->m_appliedTorqueBodyA[0], feedback->m_appliedTorqueBodyA[1], feedback->m_appliedTorqueBodyA[2];
        return r;
    }

    Eigen::VectorXf BulletRobot::getForceTorqueFeedbackB(const LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf r(6);
        r.setZero();

        if (!link.joint || !link.joint->needsFeedback())
        {
            return r;
        }

        btJointFeedback* feedback = link.joint->getJointFeedback();

        if (!feedback)
        {
            return r;
        }

        r << feedback->m_appliedForceBodyB[0], feedback->m_appliedForceBodyB[1], feedback->m_appliedForceBodyB[2], feedback->m_appliedTorqueBodyB[0], feedback->m_appliedTorqueBodyB[1], feedback->m_appliedTorqueBodyB[2];
        return r;
    }

    Eigen::VectorXf BulletRobot::getJointForceTorqueGlobal(const BulletRobot::LinkInfo& link)
    {
        MutexLockPtr lock = getScopedLock();
        Eigen::VectorXf ftA = getForceTorqueFeedbackA(link);
        Eigen::VectorXf ftB = getForceTorqueFeedbackB(link);

        Eigen::Vector3f jointGlobal = link.nodeJoint->getGlobalPose().block(0, 3, 3, 1);
        Eigen::Vector3f comBGlobal = link.nodeB->getCoMGlobal();

        // force that is applied on objectA by objectB -> so the force that object B applies on the joint
        Eigen::Vector3f forceOnBGlobal =  ftB.head(3);

        Eigen::Vector3f torqueBGlobal =  ftB.tail(3);

        // the lever from Object B CoM to Joint
        Eigen::Vector3f leverOnJoint = (comBGlobal - jointGlobal) * 0.001f * BulletObject::ScaleFactor;
        // Calculate the torque in Joint by taking the torque that presses on the CoM of BodyB and the Torque of BodyB on the joint
        // forceOnBGlobal is inverted in next line because it is the force of A on B to hold it in position
        // torqueBGlobal is inverted in next line because it is the torque on B from A to compensate torque of other objects (which is the torque we would like) to hold it in place and therefore needs to be inverted as well
        Eigen::Vector3f torqueJointGlobal = (leverOnJoint).cross(-forceOnBGlobal)  + (-1) * torqueBGlobal;
        Eigen::VectorXf result(6);
        result.head(3) = ftA.head(3); // force in joint is same as force on CoM of A
        result.tail(3) = torqueJointGlobal;
        return result / BulletObject::MassFactor;
    }

    void BulletRobot::setMaximumMotorImpulse(double maxImpulse)
    {
        bulletMaxMotorImulse = (btScalar)maxImpulse;
    }

    double BulletRobot::getMaximumMotorImpulse() const
    {
        return static_cast<double>(bulletMaxMotorImulse);
    }

} // namespace VirtualRobot
